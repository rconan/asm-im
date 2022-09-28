use arrow::Arrow;
use dos_actors::{
    io::{Data, Read, UniqueIdentifier, Write},
    prelude::*,
    Size, Update, UID,
};
use fem::FEM;
use geotrans::{Segment, SegmentTrait, Transform, M1};
use std::sync::Arc;
use vec_box::vec_box;
use windloads::M1Loads;

pub struct ForcesToOSS {
    data: Arc<Data<M1Loads>>,
}

impl ForcesToOSS {
    pub fn new() -> Self {
        Self {
            data: Arc::new(Data::new(Vec::new())),
        }
    }
}
#[derive(UID)]
pub enum M1ForcesOSS {}
impl Update for ForcesToOSS {}
impl Read<M1Loads> for ForcesToOSS {
    fn read(&mut self, data: Arc<Data<M1Loads>>) {
        self.data = Arc::clone(&data);
    }
}
impl Write<M1ForcesOSS> for ForcesToOSS {
    fn write(&mut self) -> Option<Arc<Data<M1ForcesOSS>>> {
        let data: &[f64] = &self.data;
        let fm: Vec<f64> = data
            .chunks(6)
            .enumerate()
            .flat_map(|(sid, fm)| {
                let segment = Segment::<M1>::new(sid as i32 + 1);
                let mut foss_m = fm[..3].to_vec().vtov(segment).unwrap();
                foss_m.extend_from_slice(&fm[3..]);
                foss_m
            })
            .collect();
        Some(Arc::new(Data::new(fm)))
    }
}
impl Size<M1ForcesOSS> for ForcesToOSS {
    fn len(&self) -> usize {
        42
    }
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let sim_duration = 30_usize;
    let sim_sampling_frequency = 8000;
    let cfd_case = "/fsx/CASES/zen30az000_CD17";

    println!("Analyzing: {cfd_case}");

    std::env::set_var(
        "DATA_REPO",
        std::path::Path::new(&std::env::var("CARGO_MANIFEST_DIR")?)
            .join("src")
            .join("bin")
            .join("analysis"),
    );

    let mut fem = FEM::from_env()?;

    let cfd_loads_client = windloads::CfdLoads::foh(cfd_case, sim_sampling_frequency)
        .duration(sim_duration as f64)
        .mount(&mut fem, 0, None)
        .m1_segments()
        .m2_segments()
        .build()?
        .into_arcx();

    let mut cfd_loads: Initiator<_> = Actor::new(cfd_loads_client.clone()).name("CFD Loads");
    let mut logs: Terminator<_> = (
        Arrow::builder(sim_duration * sim_sampling_frequency).build(),
        "Data Logs",
    )
        .into();
    let mut oss_forces: Actor<_> = (ForcesToOSS::new(), "Conversion of Forces to OSS").into();

    cfd_loads
        .add_output()
        .multiplex(2)
        .unbounded()
        .build::<M1Loads>()
        .into_input(&mut oss_forces)
        .log(&mut logs)
        .await;
    oss_forces
        .add_output()
        .unbounded()
        .build::<M1ForcesOSS>()
        .log(&mut logs)
        .await;

    Model::new(vec_box![cfd_loads, logs, oss_forces])
        .name("CFDLOADS")
        .check()?
        .flowchart()
        .run()
        .wait()
        .await?;

    Ok(())
}
