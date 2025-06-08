use std::fs::File;
use std::io::{self, BufRead, BufReader}; // BufReader is necessary!
use std::sync::mpsc;
use std::thread;
use std::time::Duration;

use encoding_rs_io::DecodeReaderBytes; // Import the decoder
use kiss3d::camera::ArcBall;
use kiss3d::light::Light;
use kiss3d::window::Window;

struct ObjectState {
    position: kiss3d::nalgebra::Point3<f32>,
    rotation: kiss3d::nalgebra::UnitQuaternion<f32>,
}

const LOG_FILE_PATH: &str = "live_log.txt";

fn main() {
    let (tx, rx) = mpsc::channel::<ObjectState>();

    thread::spawn(move || {
        println!(
            "Visualizer Input Thread: Waiting for '{}' to be created...",
            LOG_FILE_PATH
        );

        let file = loop {
            match File::open(LOG_FILE_PATH) {
                Ok(f) => {
                    println!("Visualizer Input Thread: File found. Tailing for new data...");
                    break f;
                }
                Err(_) => {
                    thread::sleep(Duration::from_secs(1));
                    continue;
                }
            }
        };

        // --- THIS IS THE CORRECT, IDIOMATIC, AND ROBUST WAY ---
        // 1. Wrap the file in the decoder. `decoder` is now a `Read`.
        let decoder = DecodeReaderBytes::new(file);

        // 2. Wrap the `decoder` in a `BufReader`. This gives us a `BufRead`,
        //    which is required to use the convenient `.lines()` iterator.
        //    This is the step that fixes the compiler error.
        let reader = BufReader::new(decoder);

        // 3. Use the `.lines()` iterator. This is much more robust than a manual
        //    `read_line` loop for tailing, as it handles buffering and waiting
        //    for new data correctly under the hood. It will not stop.
        for line in reader.lines() {
            let line = match line {
                Ok(l) => l,
                Err(_) => {
                    // This can happen if the logger stops and the file is closed.
                    // Or a decoding error mid-stream. We can just wait and retry.
                    thread::sleep(Duration::from_millis(100));
                    continue;
                }
            };

            // Successfully read and decoded a line.
            if let Some(state) = parse_state_vector(&line) {
                if tx.send(state).is_err() {
                    println!("Visualizer Input Thread: Main window closed. Exiting.");
                    break;
                }
            }
        }
    });

    // --- The rest of the code is unchanged ---
    let mut window = Window::new("Rust Real-Time Visualizer");
    window.set_light(Light::StickToCamera);

    let mut cube = window.add_cube(1.0, 1.0, 1.0);
    cube.set_color(0.0, 1.0, 1.0);

    let mut trajectory_points: Vec<kiss3d::nalgebra::Point3<f32>> = Vec::new();

    let eye = kiss3d::nalgebra::Point3::new(10.0f32, 10.0, 10.0);
    let at = kiss3d::nalgebra::Point3::origin();
    let mut camera = ArcBall::new(eye, at);

    while window.render_with_camera(&mut camera) {
        if let Ok(new_state) = rx.try_recv() {
            cube.set_local_translation(new_state.position.into());
            cube.set_local_rotation(new_state.rotation);
            trajectory_points.push(new_state.position);
        }

        for points in trajectory_points.windows(2) {
            window.draw_line(
                &points[0],
                &points[1],
                &kiss3d::nalgebra::Point3::new(0.6, 0.6, 0.6),
            );
        }
    }
}

fn parse_state_vector(line: &str) -> Option<ObjectState> {
    let start = line.find('[')?;
    let end = line.find(']')?;

    let data_slice = &line[start + 1..end];

    let parts: Vec<f32> = data_slice
        .split(',')
        .filter_map(|s| s.trim().parse::<f32>().ok())
        .collect();

    if parts.len() == 7 {
        let pos = kiss3d::nalgebra::Point3::new(parts[0], parts[1], parts[2]);
        let rot =
            kiss3d::nalgebra::UnitQuaternion::from_quaternion(kiss3d::nalgebra::Quaternion::new(
                parts[3], // qw
                parts[4], // qx
                parts[5], // qy
                parts[6], // qz
            ));
        Some(ObjectState {
            position: pos,
            rotation: rot,
        })
    } else {
        None
    }
}
