use std::path::Path;

use arrow::Arrow;
use asms::FEM_MODAL_DAMPING;
use dos_actors::prelude::*;
use dos_actors::{io::UniqueIdentifier, UID};
use fem::{
    dos::{DiscreteModalSolver, ExponentialMatrix},
    fem_io::*,
    FEM,
};
use lom::{TipTilt, LOM};
use mount::{Mount, MountEncoders, MountSetPoint, MountTorques};
use vec_box::vec_box;

#[derive(UID)]
#[alias(
    name = "lom::actors_interface::TipTilt",
    client = "LOM",
    traits = "Write"
)]
pub enum Jitter2ms {}
#[derive(UID)]
#[alias(
    name = "lom::actors_interface::TipTilt",
    client = "LOM",
    traits = "Write"
)]
pub enum Jitter7ms {}
#[derive(UID)]
#[alias(
    name = "lom::actors_interface::TipTilt",
    client = "LOM",
    traits = "Write"
)]
pub enum Jitter12ms {}
#[derive(UID)]
#[alias(
    name = "lom::actors_interface::TipTilt",
    client = "LOM",
    traits = "Write"
)]
pub enum Jitter17ms {}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let sim_duration = 3_usize;
    let sim_sampling_frequency = 8000;
    let n_step = sim_duration * sim_sampling_frequency;

    let cfd_root = Path::new("/fsx/CASES");
    std::env::set_var(
        "DATA_REPO",
        std::path::Path::new(&std::env::var("CARGO_MANIFEST_DIR")?)
            .join("src")
            .join("bin")
            .join("gusts"),
    );
    std::env::set_var("LOM", "/fsx");

    let arrow = Arrow::builder(n_step).build().into_arcx();

    for wind_speed in [2, 7, 12, 17].into_iter().take(1) {
        let cfd_case = if wind_speed <= 7 {
            format!("zen30az000_OS{wind_speed}")
        } else {
            format!("zen30az000_CD{wind_speed}")
        };
        println!("CFD CASE: {cfd_case}");
        let cfd_path = cfd_root.join(&cfd_case);

        let mut fem = FEM::from_env()?.static_from_env()?;
        let n_io = (fem.n_inputs(), fem.n_outputs());

        let cfd_loads_client =
            windloads::CfdLoads::foh(cfd_path.to_str().unwrap(), sim_sampling_frequency)
                .duration(sim_duration as f64)
                .mount(&mut fem, 0, None)
                .m1_segments()
                .m2_segments()
                .build()?
                .into_arcx();

        let state_space = DiscreteModalSolver::<ExponentialMatrix>::from_fem(fem)
            .sampling(sim_sampling_frequency as f64)
            .proportional_damping(FEM_MODAL_DAMPING)
            .use_static_gain_compensation(n_io)
            .ins::<OSSElDriveTorque>()
            .ins::<OSSAzDriveTorque>()
            .ins::<OSSRotDriveTorque>()
            .ins::<OSSM1Lcl6F>()
            .ins::<MCM2Lcl6F>()
            .ins::<CFD2021106F>()
            .outs::<OSSAzEncoderAngle>()
            .outs::<OSSElEncoderAngle>()
            .outs::<OSSRotEncoderAngle>()
            .outs::<OSSM1Lcl>()
            .outs::<MCM2Lcl6D>()
            .build()?
            .into_arcx();
        let mut fem: Actor<_> = Actor::new(state_space.clone()).name(format!(
            "FEM w/ ASM:
        {:}",
            std::env::var("FEM_REPO").unwrap()
        ));

        let mut cfd_loads: Initiator<_> = Actor::new(cfd_loads_client.clone()).name(cfd_case);
        let mut logs: Terminator<_> = Actor::new(arrow.clone()).name("Data Logs");

        cfd_loads
            .add_output()
            .unbounded()
            .build::<OSSM1Lcl6F>()
            .into_input(&mut fem);
        cfd_loads
            .add_output()
            .unbounded()
            .build::<MCM2Lcl6F>()
            .into_input(&mut fem);
        cfd_loads
            .add_output()
            .unbounded()
            .build::<CFD2021106F>()
            .into_input(&mut fem);

        let mut mount: Actor<_> = (Mount::new(), "Mount Control System").into();
        let mut mount_set_point: Initiator<_> =
            (Signals::new(3, n_step).progress(), "Mount Set Point").into();
        mount_set_point
            .add_output()
            .build::<MountSetPoint>()
            .into_input(&mut mount);
        mount
            .add_output()
            .build::<MountTorques>()
            .into_input(&mut fem);
        fem.add_output()
            .bootstrap()
            .build::<MountEncoders>()
            .into_input(&mut mount);

        let mut lom: Actor<_> = (LOM::builder().build()?, "Linear Optical Model").into();
        fem.add_output()
            .unbounded()
            .build::<OSSM1Lcl>()
            .into_input(&mut lom);
        fem.add_output()
            .unbounded()
            .build::<MCM2Lcl6D>()
            .into_input(&mut lom);
        match wind_speed {
            v if v == 2 => {
                lom.add_output()
                    .unbounded()
                    .build::<Jitter2ms>()
                    .logn(&mut logs, 2)
                    .await;
            }
            v if v == 7 => {
                lom.add_output()
                    .unbounded()
                    .build::<Jitter7ms>()
                    .logn(&mut logs, 2)
                    .await;
            }
            v if v == 12 => {
                lom.add_output()
                    .unbounded()
                    .build::<Jitter12ms>()
                    .logn(&mut logs, 2)
                    .await;
            }
            v if v == 17 => {
                lom.add_output()
                    .unbounded()
                    .build::<Jitter17ms>()
                    .logn(&mut logs, 2)
                    .await;
            }
            _ => unimplemented!(),
        }

        Model::new(vec_box![cfd_loads, logs, fem, mount_set_point, mount, lom])
            .name("CFD GUSTS")
            .check()?
            .flowchart()
            .run()
            .await?;
    }

    Ok(())
}
