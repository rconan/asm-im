use dos_actors::{
    clients::{
        arrow_client::{Arrow,FileFormat,MatFormat,Get},
        asm::*,
        m1::*,
        mount::{Mount, MountEncoders, MountSetPoint, MountTorques},
        windloads::{self, M1Loads, M2Loads, MountLoads},
        Smooth, Weight,
    },
    io::{Data, Read, Write},
    prelude::*,
    Update,
};
use fem::{
    dos::{DiscreteModalSolver, ExponentialMatrix},
    fem_io::*,
    FEM,
};
use lom::{Stats, LOM};
use std::{env, sync::Arc,path::Path};
use vec_box::vec_box;
        use matio_rs::{MatFile, MatVar, Save};

#[derive(Default)]
pub struct Adder {
    m2_loads: Option<Arc<Data<M2Loads>>>,
    u_cp: Option<Arc<Data<Ucp>>>,
}
impl Update for Adder {}
impl Read<M2Loads> for Adder {
    fn read(&mut self, data: Arc<Data<M2Loads>>) {
        self.m2_loads = Some(data.clone());
    }
}
impl Read<Ucp> for Adder {
    fn read(&mut self, data: Arc<Data<Ucp>>) {
        self.u_cp = Some(data.clone());
    }
}
impl Write<MCM2Lcl6F> for Adder {
    fn write(&mut self) -> Option<Arc<Data<MCM2Lcl6F>>> {
        if let (Some(m2_loads), Some(u_cp)) = (self.m2_loads.as_ref(), self.u_cp.as_ref()) {
            let sum: Vec<f64> = m2_loads
                .iter()
                .zip(u_cp.iter())
                .map(|(&m2_loads, &u_cp)| m2_loads + u_cp)
                .collect();
            Some(Arc::new(Data::new(sum)))
        } else {
            None
        }
    }
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let sim_sampling_frequency = 8000;
    let sim_duration = 60_usize;
    let n_step = sim_sampling_frequency * sim_duration;

    let mut fem = FEM::from_env()?.static_from_env()?;
    let n_io = (fem.n_inputs(), fem.n_outputs());
    println!("{:}", fem);

    let mut cfd_loads: Initiator<_> = (
        windloads::CfdLoads::foh("/fsx/CASES/zen30az000_OS7", sim_sampling_frequency)
            .duration(sim_duration as f64)
            .mount(&mut fem, 0, None)
            .m1_segments()
            .m2_segments()
            .build()?,
        "CFD Wind loads",
    )
        .into();

    let state_space = DiscreteModalSolver::<ExponentialMatrix>::from_fem(fem)
        .sampling(sim_sampling_frequency as f64)
        .proportional_damping(0.5 / 100.)
        .ins::<OSSElDriveTorque>()
        .ins::<OSSAzDriveTorque>()
        .ins::<OSSRotDriveTorque>()
        .ins::<OSSHarpointDeltaF>()
        .ins::<M1ActuatorsSegment1>()
        .ins::<M1ActuatorsSegment2>()
        .ins::<M1ActuatorsSegment3>()
        .ins::<M1ActuatorsSegment4>()
        .ins::<M1ActuatorsSegment5>()
        .ins::<M1ActuatorsSegment6>()
        .ins::<M1ActuatorsSegment7>()
        .ins::<OSSM1Lcl6F>()
        .ins::<MCM2SmHexF>()
        .ins::<MCM2CP6F>()
        .ins::<MCM2RB6F>()
        .ins::<MCM2Lcl6F>()
        .ins::<CFD2021106F>()
        .outs::<OSSAzEncoderAngle>()
        .outs::<OSSElEncoderAngle>()
        .outs::<OSSRotEncoderAngle>()
        .outs::<OSSHardpointD>()
        .outs::<MCM2SmHexD>()
        .outs::<OSSM1Lcl>()
        .outs::<MCM2Lcl6D>()
        .outs::<MCM2RB6D>()
        .outs::<OSSTrussIF6D>()
        .use_static_gain_compensation(n_io)
        .build()?;
    println!("{state_space}");

    // FEM
    let mut fem: Actor<_> = (
        state_space,
        format!(
            "FEM w/ ASM:
        {:}",
            env::var("FEM_REPO").unwrap()
        ),
    )
        .into();

    let signal: std::result::Result<OneSignal,_> = Signals::new(1,n_step).output_signal(0,Signal::Sigmoid{amplitude:1f64,sampling_frequency_hz:sim_sampling_frequency as f64}).progress().into();
    let mut sigmoid: Initiator<OneSignal,1> = 
    (signal?,"Sigmoid").into();
    let mut smooth_m1_loads: Actor<_> = Smooth::new().into();
    let mut smooth_m2_loads: Actor<_> = Smooth::new().into();
    let mut smooth_mount_loads: Actor<_> = Smooth::new().into();
    let mut adder: Actor<_> = Adder::default().into();

    let decimation = 8;
    let logging = Arrow::builder(n_step).decimation(decimation)
    //.file_format(FileFormat::Matlab(MatFormat::TimeBased(sim_sampling_frequency as f64)))
    .build().into_arcx();
    let mut sink = Terminator::<_>::new(logging.clone());

    sigmoid
        .add_output()
        .multiplex(3)
        .build::<Weight>()
        .into_input(&mut smooth_m1_loads)
        .into_input(&mut smooth_m2_loads)
        .into_input(&mut smooth_mount_loads)
        .confirm()?;
    cfd_loads
        .add_output()
        .build::<M1Loads>()
        .into_input(&mut smooth_m1_loads);
    smooth_m1_loads
        .add_output()
        .build::<OSSM1Lcl6F>()
        .into_input(&mut fem);
    cfd_loads
        .add_output()
        .build::<M2Loads>()
        .into_input(&mut smooth_m2_loads);
    smooth_m2_loads
        .add_output()
        .build::<M2Loads>()
        .into_input(&mut adder);
    adder.add_output().build::<MCM2Lcl6F>().into_input(&mut fem);
    cfd_loads
        .add_output()
        .build::<MountLoads>()
        .into_input(&mut smooth_mount_loads);
    smooth_mount_loads
        .add_output()
        .build::<CFD2021106F>()
        .into_input(&mut fem);
    // MOUNT
    let mut mount: Actor<_> = Mount::new().into();

    const M1_RATE: usize = 80;
    assert_eq!(sim_sampling_frequency / M1_RATE, 100);

    // HARDPOINTS
    let mut m1_hardpoints: Actor<_> = m1_ctrl::hp_dynamics::Controller::new().into();
    // LOADCELLS
    let mut m1_hp_loadcells: Actor<_, 1, M1_RATE> =
        m1_ctrl::hp_load_cells::Controller::new().into();
    // M1 SEGMENTS ACTUATORS
    let mut m1_segment1: Actor<_, M1_RATE, 1> =
        m1_ctrl::actuators::segment1::Controller::new().into();
    let mut m1_segment2: Actor<_, M1_RATE, 1> =
        m1_ctrl::actuators::segment2::Controller::new().into();
    let mut m1_segment3: Actor<_, M1_RATE, 1> =
        m1_ctrl::actuators::segment3::Controller::new().into();
    let mut m1_segment4: Actor<_, M1_RATE, 1> =
        m1_ctrl::actuators::segment4::Controller::new().into();
    let mut m1_segment5: Actor<_, M1_RATE, 1> =
        m1_ctrl::actuators::segment5::Controller::new().into();
    let mut m1_segment6: Actor<_, M1_RATE, 1> =
        m1_ctrl::actuators::segment6::Controller::new().into();
    let mut m1_segment7: Actor<_, M1_RATE, 1> =
        m1_ctrl::actuators::segment7::Controller::new().into();

    let mut mount_set_point: Initiator<_> = (Signals::new(3, n_step), "Mount Set Point").into();
    mount_set_point
        .add_output()
        .build::<MountSetPoint>()
        .into_input(&mut mount);
    mount
        .add_output()
        .build::<MountTorques>()
        .into_input(&mut fem);

    let mut m1rbm_set_point: Initiator<_> = (Signals::new(42, n_step), "M1 RBM Set Point").into();
    m1rbm_set_point
        .add_output()
        .build::<M1RBMcmd>()
        .into_input(&mut m1_hardpoints);
    m1_hardpoints
        .add_output()
        .multiplex(2)
        .build::<OSSHarpointDeltaF>()
        .into_input(&mut fem)
        .into_input(&mut m1_hp_loadcells);

    m1_hp_loadcells
        .add_output()
        .build::<S1HPLC>()
        .into_input(&mut m1_segment1);
    m1_hp_loadcells
        .add_output()
        .build::<S2HPLC>()
        .into_input(&mut m1_segment2);
    m1_hp_loadcells
        .add_output()
        .build::<S3HPLC>()
        .into_input(&mut m1_segment3);
    m1_hp_loadcells
        .add_output()
        .build::<S4HPLC>()
        .into_input(&mut m1_segment4);
    m1_hp_loadcells
        .add_output()
        .build::<S5HPLC>()
        .into_input(&mut m1_segment5);
    m1_hp_loadcells
        .add_output()
        .build::<S6HPLC>()
        .into_input(&mut m1_segment6);
    m1_hp_loadcells
        .add_output()
        .build::<S7HPLC>()
        .into_input(&mut m1_segment7);

    m1_segment1
        .add_output()
        .bootstrap()
        .build::<M1ActuatorsSegment1>()
        .into_input(&mut fem);
    m1_segment2
        .add_output()
        .bootstrap()
        .build::<M1ActuatorsSegment2>()
        .into_input(&mut fem);
    m1_segment3
        .add_output()
        .bootstrap()
        .build::<M1ActuatorsSegment3>()
        .into_input(&mut fem);
    m1_segment4
        .add_output()
        .bootstrap()
        .build::<M1ActuatorsSegment4>()
        .into_input(&mut fem);
    m1_segment5
        .add_output()
        .bootstrap()
        .build::<M1ActuatorsSegment5>()
        .into_input(&mut fem);
    m1_segment6
        .add_output()
        .bootstrap()
        .build::<M1ActuatorsSegment6>()
        .into_input(&mut fem);
    m1_segment7
        .add_output()
        .bootstrap()
        .build::<M1ActuatorsSegment7>()
        .into_input(&mut fem);

    // M2 POSITIONER COMMAND
    let mut m2_pos_cmd: Initiator<_> =
        (Signals::new(42, n_step), "M2 Positionner Set Point").into();
    // FSM POSITIONNER
    let mut m2_positionner: Actor<_> =
        (m2_ctrl::positionner::Controller::new(), "M2 Positionner").into();
    m2_pos_cmd
        .add_output()
        .build::<M2poscmd>()
        .into_input(&mut m2_positionner);
    m2_positionner
        .add_output()
        .build::<MCM2SmHexF>()
        .into_input(&mut fem);
    // ASM SET POINT
    let mut asm_cmd: Initiator<_> = (Signals::new(21, n_step), "ASMS Set Point").into();
    // ASM INNER CONTROLLER
    let mut asm_inner: Actor<_> = (
        m2_ctrl::ptt_fluid_damping::Controller::new(),
        "ASMS
        piston,tip,tilt & fluid damping",
    )
        .into();
    asm_cmd
        .add_output()
        .build::<Rrbfs>()
        .into_input(&mut asm_inner);
    asm_inner
        .add_output()
        .build::<MCM2Lcl6F>()
        .into_input(&mut fem);
    asm_inner
        .add_output()
        .build::<MCM2RB6F>()
        .into_input(&mut fem);
    asm_inner.add_output().build::<Ucp>().into_input(&mut adder);

    fem.add_output()
        .bootstrap()
        .multiplex(2)
        .build::<MountEncoders>()
        .into_input(&mut mount)
        .logn(&mut sink,14).await
        .confirm()?;
    fem.add_output()
        .bootstrap()
        .build::<OSSHardpointD>()
        .into_input(&mut m1_hp_loadcells);
    fem.add_output()
        .bootstrap()
        .build::<OSSM1Lcl>()
        .log(&mut sink)
        .await;    
    fem.add_output()
        .bootstrap()
        .build::<OSSTrussIF6D>()
        .logn(&mut sink,18)
        .await;
    fem.add_output()
        .multiplex(2)
        .bootstrap()
        .build::<MCM2Lcl6D>()
        .into_input(&mut asm_inner)
        .log(&mut sink)
        .await
        .confirm()?;
    fem.add_output()
        .bootstrap()
        .build::<MCM2SmHexD>()
        .into_input(&mut m2_positionner);
    fem.add_output()
        .bootstrap()
        .build::<MCM2RB6D>()
        .into_input(&mut asm_inner)
        .confirm()?;

    Model::new(vec_box![
        mount_set_point,
        mount,
        m1rbm_set_point,
        m1_hardpoints,
        m1_hp_loadcells,
        m1_segment1,
        m1_segment2,
        m1_segment3,
        m1_segment4,
        m1_segment5,
        m1_segment6,
        m1_segment7,
        m2_pos_cmd,
        m2_positionner,
        asm_cmd,
        asm_inner,
        fem,
        sink,
        cfd_loads,
        sigmoid,
        smooth_m1_loads,
        smooth_m2_loads,
        adder,
        smooth_mount_loads
    ])
    .name("asms")
    .flowchart()
    .check()?
    .run()
    .wait()
    .await?;

    let logger = &mut *logging.lock().await;
    let m1_rbm: Vec<Vec<f64>> = logger.get("OSSM1Lcl")?;
    let m2_rbm: Vec<Vec<f64>> = logger.get("MCM2Lcl6D")?;
    let truss_if: Vec<Vec<f64>> = logger.get("OSSTrussIF6D")?;

    let n_sample = m1_rbm.len();
    let n_data = m1_rbm[0].len() + m2_rbm[0].len() + truss_if[0].len();

    let  data: Vec<f64> = m1_rbm.into_iter()
    .zip(m2_rbm.into_iter())
    .zip(truss_if.into_iter())
    .flat_map(|((a,b),c)| a.into_iter().chain(b.into_iter()).chain(c.into_iter()).collect::<Vec<f64>>()).collect();
    let tau = ((sim_sampling_frequency/decimation) as f64).recip();
    let time: Vec<_> = (0..n_sample).map(|i| tau * i as f64).collect();

    let root = Path::new("v1_wind_phasing_2022_zen30az000_OS7").with_extension("mat");
    let mat_file = MatFile::save(&root)?;
    mat_file.write(MatVar::<Vec<f64>>::new("tt",time.as_slice())?);
    mat_file.write(MatVar::<Vec<f64>>::array("yt",data.as_slice(),(n_data,n_sample))?);

/*     let lom = LOM::builder()
        .rigid_body_motions_record((*logging.lock().await).record()?,Some("OSSM1Lcl"),Some("MCM2Lcl6D"))?
        .build()?;
    let tiptilt = lom.tiptilt_mas();
    let n_sample = 1000;
    let tt = tiptilt.std(Some(n_sample));
    println!("TT STD.: {:.3?}mas", tt); */


    Ok(())
}
