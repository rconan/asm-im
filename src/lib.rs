use arrow::Arrow;
use dos_actors::{
    clients::{Smooth, Weight},
    io::{Data, Read, Write},
    prelude::*,
    Update,
};
use dos_clients_io::*;
use fem::{
    dos::{DiscreteModalSolver, ExponentialMatrix},
    fem_io::*,
    FEM,
};
use m1_ctrl::*;
use m2_ctrl::*;
use mount::Mount;
use nalgebra as na;
use std::{collections::HashMap, env, fs::File, path::Path, sync::Arc};
use vec_box::vec_box;

#[cfg(feature = "damping0005")]
pub const FEM_MODAL_DAMPING: f64 = 0.005;
#[cfg(feature = "damping002")]
pub const FEM_MODAL_DAMPING: f64 = 0.02;

#[derive(Default)]
pub struct Adder {
    m2_loads: Option<Arc<Data<CFDM2WindLoads>>>,
    u_cp: Option<Arc<Data<Ucp>>>,
}
impl Update for Adder {}
impl Read<CFDM2WindLoads> for Adder {
    fn read(&mut self, data: Arc<Data<CFDM2WindLoads>>) {
        self.m2_loads = Some(data.clone());
    }
}
impl Read<Ucp> for Adder {
    fn read(&mut self, data: Arc<Data<Ucp>>) {
        self.u_cp = Some(data.clone());
    }
}
impl Write<CFDM2WindLoads> for Adder {
    fn write(&mut self) -> Option<Arc<Data<CFDM2WindLoads>>> {
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

fn fig_2_mode(sid: u32) -> na::DMatrix<f64> {
    let root_env = env::var("M1CALIBRATION").unwrap_or_else(|_| ".".to_string());
    let root = Path::new(&root_env);
    let fig_2_mode: Vec<f64> =
        bincode::deserialize_from(File::open(root.join(format!("m1s{sid}fig2mode.bin"))).unwrap())
            .unwrap();
    if sid < 7 {
        na::DMatrix::from_vec(162, 602, fig_2_mode)
    } else {
        na::DMatrix::from_vec(151, 579, fig_2_mode).insert_rows(151, 11, 0f64)
    }
}

pub async fn model(cfd_case: &str) -> anyhow::Result<()> {
    let sim_sampling_frequency = 8000;
    let sim_duration = 400_usize;
    let n_step = sim_sampling_frequency * sim_duration;
    let n_part = 10;

    let mut fem = FEM::from_env()?.static_from_env()?;
    let n_io = (fem.n_inputs(), fem.n_outputs());
    println!("{:}", fem);

    // --------------------------------------
    // CLIENTS

    let cfd_loads_client = windloads::CfdLoads::foh(cfd_case, sim_sampling_frequency)
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
        .outs_with::<M1Segment1AxialD>(fig_2_mode(1))
        .outs_with::<M1Segment2AxialD>(fig_2_mode(2))
        .outs_with::<M1Segment3AxialD>(fig_2_mode(3))
        .outs_with::<M1Segment4AxialD>(fig_2_mode(4))
        .outs_with::<M1Segment5AxialD>(fig_2_mode(5))
        .outs_with::<M1Segment6AxialD>(fig_2_mode(6))
        .outs_with::<M1Segment7AxialD>(fig_2_mode(7))
        .outs::<OSSM1EdgeSensors>()
        .build()?
        .into_arcx();
    //println!("{state_space}");

    let signal = {
        let signal: std::result::Result<OneSignal, _> = Signals::new(1, n_step)
            .output_signal(
                0,
                Signal::Sigmoid {
                    amplitude: 1f64,
                    sampling_frequency_hz: sim_sampling_frequency as f64,
                },
            )
            .progress()
            .into();
        signal?.into_arcx()
    };
    let m1_smoother = Smooth::new().into_arcx();
    let m2_smoother = Smooth::new().into_arcx();
    let mount_smoother = Smooth::new().into_arcx();
    let adder_client = Adder::default().into_arcx();

    let mount_client = Mount::new().into_arcx();
    let m1_hardpoints_client = m1_ctrl::hp_dynamics::Controller::new().into_arcx();
    let m1_hp_loadcells_client = m1_ctrl::hp_load_cells::Controller::new().into_arcx();

    let m1_client_segment1 = m1_ctrl::actuators::segment1::Controller::new().into_arcx();
    let m1_client_segment2 = m1_ctrl::actuators::segment2::Controller::new().into_arcx();
    let m1_client_segment3 = m1_ctrl::actuators::segment3::Controller::new().into_arcx();
    let m1_client_segment4 = m1_ctrl::actuators::segment4::Controller::new().into_arcx();
    let m1_client_segment5 = m1_ctrl::actuators::segment5::Controller::new().into_arcx();
    let m1_client_segment6 = m1_ctrl::actuators::segment6::Controller::new().into_arcx();
    let m1_client_segment7 = m1_ctrl::actuators::segment7::Controller::new().into_arcx();

    let mount_set_point_client = Signals::new(3, n_step).into_arcx();
    let m1rbm_set_point_client = Signals::new(42, n_step).into_arcx();

    let m2_pos_cmd_client = Signals::new(42, n_step).into_arcx();
    let m2_positionner_client = m2_ctrl::positionner::Controller::new().into_arcx();

    let asm_cmd_client = Signals::new(21, n_step).into_arcx();
    let asm_inner_client = m2_ctrl::ptt_fluid_damping::Controller::new().into_arcx();

    let mut meta_data: HashMap<String, String> = HashMap::new();
    meta_data.insert(
        "sim_sampling_frequency".to_string(),
        format!("{sim_sampling_frequency}"),
    );
    meta_data.insert("sim_duration".to_string(), format!("{sim_duration}"));
    meta_data.insert("FEM".to_string(), env::var("FEM_REPO").unwrap());
    meta_data.insert("CFD_CASE".to_string(), cfd_case.to_string());

    // -----------------------------------
    // MODEL

    for part in 0..n_part {
        (*cfd_loads_client.lock().await).start_from(part * n_step / n_part);
        (*cfd_loads_client.lock().await).stop_after((part + 1) * n_step / n_part);

        let mut cfd_loads: Initiator<_> =
            Actor::new(cfd_loads_client.clone()).name("CFD Wind loads");

        // FEM
        let mut fem: Actor<_> = Actor::new(state_space.clone()).name(format!(
            "FEM w/ ASM:
        {:}",
            env::var("FEM_REPO").unwrap()
        ));

        let mut sigmoid: Initiator<OneSignal, 1> = Actor::new(signal.clone()).name("Sigmoid");
        let mut smooth_m1_loads: Actor<_> = Actor::new(m1_smoother.clone());
        let mut smooth_m2_loads: Actor<_> = Actor::new(m2_smoother.clone());
        let mut smooth_mount_loads: Actor<_> = Actor::new(mount_smoother.clone());
        let mut adder: Actor<_> = Actor::new(adder_client.clone());

        let decimation = 4;
        let mut sink: Terminator<_> = Arrow::builder(n_step)
            .metadata(meta_data.clone())
            .filename(format!("asms-im_windloading-part{part}"))
            .decimation(decimation)
            .build()
            .into();

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
            .build::<CFDM1WindLoads>()
            .into_input(&mut smooth_m1_loads);
        smooth_m1_loads
            .add_output()
            .build::<CFDM1WindLoads>()
            .into_input(&mut fem);
        cfd_loads
            .add_output()
            .build::<CFDM2WindLoads>()
            .into_input(&mut smooth_m2_loads);
        smooth_m2_loads
            .add_output()
            .build::<CFDM2WindLoads>()
            .into_input(&mut adder);
        adder
            .add_output()
            .build::<CFDM2WindLoads>()
            .into_input(&mut fem);
        cfd_loads
            .add_output()
            .build::<CFDMountWindLoads>()
            .into_input(&mut smooth_mount_loads);
        smooth_mount_loads
            .add_output()
            .build::<CFDMountWindLoads>()
            .into_input(&mut fem);
        // MOUNT
        let mut mount: Actor<_> = Actor::new(mount_client.clone());

        const M1_RATE: usize = 80;
        assert_eq!(sim_sampling_frequency / M1_RATE, 100);

        // HARDPOINTS
        let mut m1_hardpoints: Actor<_> = Actor::new(m1_hardpoints_client.clone());
        // LOADCELLS
        let mut m1_hp_loadcells: Actor<_, 1, M1_RATE> = Actor::new(m1_hp_loadcells_client.clone());
        // M1 SEGMENTS ACTUATORS
        let mut m1_segment1: Actor<_, M1_RATE, 1> = Actor::new(m1_client_segment1.clone());
        let mut m1_segment2: Actor<_, M1_RATE, 1> = Actor::new(m1_client_segment2.clone());
        let mut m1_segment3: Actor<_, M1_RATE, 1> = Actor::new(m1_client_segment3.clone());
        let mut m1_segment4: Actor<_, M1_RATE, 1> = Actor::new(m1_client_segment4.clone());
        let mut m1_segment5: Actor<_, M1_RATE, 1> = Actor::new(m1_client_segment5.clone());
        let mut m1_segment6: Actor<_, M1_RATE, 1> = Actor::new(m1_client_segment6.clone());
        let mut m1_segment7: Actor<_, M1_RATE, 1> = Actor::new(m1_client_segment7.clone());

        let mut mount_set_point: Initiator<_> =
            Actor::new(mount_set_point_client.clone()).name("Mount Set Point");
        mount_set_point
            .add_output()
            .build::<MountSetPoint>()
            .into_input(&mut mount);
        mount
            .add_output()
            .build::<MountTorques>()
            .into_input(&mut fem);

        let mut m1rbm_set_point: Initiator<_> =
            Actor::new(m1rbm_set_point_client.clone()).name("M1 RBM Set Point");
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
            Actor::new(m2_pos_cmd_client.clone()).name("M2 Positionner Set Point");
        // FSM POSITIONNER
        let mut m2_positionner: Actor<_> =
            Actor::new(m2_positionner_client.clone()).name("M2 Positionner");
        m2_pos_cmd
            .add_output()
            .build::<M2poscmd>()
            .into_input(&mut m2_positionner);
        m2_positionner
            .add_output()
            .build::<M2PositionerForces>()
            .into_input(&mut fem);
        // ASM SET POINT
        let mut asm_cmd: Initiator<_> = Actor::new(asm_cmd_client.clone()).name("ASMS Set Point");
        // ASM INNER CONTROLLER
        let mut asm_inner: Actor<_> = Actor::new(asm_inner_client.clone()).name(
            "ASMS
        piston,tip,tilt & fluid damping",
        );
        asm_cmd
            .add_output()
            .build::<Rrbfs>()
            .into_input(&mut asm_inner);
        asm_inner
            .add_output()
            .build::<M2ASMFaceSheetForces>()
            .into_input(&mut fem);
        asm_inner
            .add_output()
            .build::<M2ASMRigidBodyForces>()
            .into_input(&mut fem);
        asm_inner.add_output().build::<Ucp>().into_input(&mut adder);

        fem.add_output()
            .bootstrap()
            .build::<MountEncoders>()
            .into_input(&mut mount);
        fem.add_output()
            .bootstrap()
            .build::<OSSHardpointD>()
            .into_input(&mut m1_hp_loadcells);
        fem.add_output()
            .bootstrap()
            .unbounded()
            .build::<M1RigidBodyMotions>()
            .log(&mut sink)
            .await;
        fem.add_output()
            .multiplex(2)
            .bootstrap()
            .unbounded()
            .build::<M2ASMFaceSheetNodes>()
            .into_input(&mut asm_inner)
            .logn(&mut sink, 42)
            .await
            .confirm()?;
        fem.add_output()
            .bootstrap()
            .build::<M2PositionerNodes>()
            .into_input(&mut m2_positionner);
        fem.add_output()
            .bootstrap()
            .build::<M2ASMRigidBodyNodes>()
            .into_input(&mut asm_inner)
            .confirm()?;
        fem.add_output()
            .bootstrap()
            .unbounded()
            .build::<M1ModeShapes>()
            .logn(&mut sink, 162 * 7)
            .await;
        fem.add_output()
            .bootstrap()
            .unbounded()
            .build::<OSSM1EdgeSensors>()
            .logn(&mut sink, 288)
            .await;

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
    }

    Ok(())
}
