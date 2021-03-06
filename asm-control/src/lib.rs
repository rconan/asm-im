//! This crate contains Rust wrappers around the Simulink C-translated ASM segment inner loop controllers
//!
//! The Simulink C version of the controllers uses static variables so, in order to differentiate each segment controller, the static variable are renamed according to the segment #.
//! The `Makefile` uses [segment1] module as a template and copy it into a new module and substitute S1 with S`ID` in each file, e.g.
//! ```
//! make new ID=5
//! ```
//! to create a `segment5` module from `segment`.

pub mod segment1;
pub mod segment2;
pub mod segment3;
pub mod segment4;
pub mod segment5;
pub mod segment6;
pub mod segment7;
/// ASM segment controller
pub enum AsmController<'a> {
    One(segment1::Controller<'a>),
    Two(segment2::Controller<'a>),
    Three(segment3::Controller<'a>),
    Four(segment4::Controller<'a>),
    Five(segment5::Controller<'a>),
    Six(segment6::Controller<'a>),
    Seven(segment7::Controller<'a>),
}

/// The ASMS control model
pub struct ASMS<'a> {
    /// The 7 ASM segments controllers
    pub controllers: Vec<AsmController<'a>>,
    /// The proportional gain on the voice coils modal forces
    pub modal_forces_gain: f64,
    /// The segments fluid damping gain
    pub fluid_damping_gain: f64,
}
impl<'a> Default for ASMS<'a> {
    fn default() -> Self {
        use AsmController::*;
        Self {
            controllers: vec![
                One(segment1::Controller::new()),
                Two(segment2::Controller::new()),
                Three(segment3::Controller::new()),
                Four(segment4::Controller::new()),
                Five(segment5::Controller::new()),
                Six(segment6::Controller::new()),
                Seven(segment7::Controller::new()),
            ],
            modal_forces_gain: 0.5,
            fluid_damping_gain: -9.1_f64,
        }
    }
}
impl<'a> ASMS<'a> {
    /// Creates a new ASMS control model
    pub fn new() -> Self {
        Default::default()
    }
    /// Sets the proportional gain on the voice coils modal forces
    pub fn modal_forces_gain(self, modal_forces_gain: f64) -> Self {
        Self {
            modal_forces_gain,
            ..self
        }
    }
}

impl<'a> From<Vec<u8>> for ASMS<'a> {
    /// Creates a new ASMS control model from a vector of segment id # in the range [1,7]
    fn from(sid: Vec<u8>) -> Self {
        use AsmController::*;
        Self {
            controllers: sid
                .into_iter()
                .map(|i| match i {
                    1 => One(segment1::Controller::new()),
                    2 => Two(segment2::Controller::new()),
                    3 => Three(segment3::Controller::new()),
                    4 => Four(segment4::Controller::new()),
                    5 => Five(segment5::Controller::new()),
                    6 => Six(segment6::Controller::new()),
                    7 => Seven(segment7::Controller::new()),
                    _ => panic!("Segment number id must in the range [1,2,...,7]"),
                })
                .collect::<Vec<AsmController>>(),
            ..Default::default()
        }
    }
}

use dosio::{io::Tags, ios, DOSIOSError, Dos, IOTags, IO};

impl<'a> IOTags for ASMS<'a> {
    fn outputs_tags(&self) -> Vec<Tags> {
        use AsmController::*;
        self.controllers
            .iter()
            .flat_map(|controller| match controller {
                One(ctrl) => ctrl.outputs_tags(),
                Two(ctrl) => ctrl.outputs_tags(),
                Three(ctrl) => ctrl.outputs_tags(),
                Four(ctrl) => ctrl.outputs_tags(),
                Five(ctrl) => ctrl.outputs_tags(),
                Six(ctrl) => ctrl.outputs_tags(),
                Seven(ctrl) => ctrl.outputs_tags(),
            })
            .collect()
    }
    fn inputs_tags(&self) -> Vec<Tags> {
        use AsmController::*;
        self.controllers
            .iter()
            .flat_map(|controller| match controller {
                One(ctrl) => ctrl.inputs_tags(),
                Two(ctrl) => ctrl.inputs_tags(),
                Three(ctrl) => ctrl.inputs_tags(),
                Four(ctrl) => ctrl.inputs_tags(),
                Five(ctrl) => ctrl.inputs_tags(),
                Six(ctrl) => ctrl.inputs_tags(),
                Seven(ctrl) => ctrl.inputs_tags(),
            })
            .collect()
    }
}
impl<'a> Dos for ASMS<'a> {
    type Input = Vec<f64>;
    type Output = Vec<f64>;

    fn outputs(&mut self) -> Option<Vec<IO<Self::Output>>> {
        use AsmController::*;
        let Self {
            modal_forces_gain,
            fluid_damping_gain,
            ..
        } = self;
        self.controllers
            .iter_mut()
            .map(|controller| match controller {
                One(ctrl) => {
                    let mut y = ctrl.outputs();
                    if let Some(ref mut v) = y {
                        v[ios!(M2S1FSCPModalF)] *= *modal_forces_gain;
                        v[ios!(M2S1FSRBModalF)] *= *fluid_damping_gain;
                    };
                    y
                }
                Two(ctrl) => {
                    let mut y = ctrl.outputs();
                    if let Some(ref mut v) = y {
                        v[ios!(M2S2FSCPModalF)] *= *modal_forces_gain;
                        v[ios!(M2S2FSRBModalF)] *= *fluid_damping_gain;
                    };
                    y
                }
                Three(ctrl) => {
                    let mut y = ctrl.outputs();
                    if let Some(ref mut v) = y {
                        v[ios!(M2S3FSCPModalF)] *= *modal_forces_gain;
                        v[ios!(M2S3FSRBModalF)] *= *fluid_damping_gain;
                    };
                    y
                }
                Four(ctrl) => {
                    let mut y = ctrl.outputs();
                    if let Some(ref mut v) = y {
                        v[ios!(M2S4FSCPModalF)] *= *modal_forces_gain;
                        v[ios!(M2S4FSRBModalF)] *= *fluid_damping_gain;
                    };
                    y
                }
                Five(ctrl) => {
                    let mut y = ctrl.outputs();
                    if let Some(ref mut v) = y {
                        v[ios!(M2S5FSCPModalF)] *= *modal_forces_gain;
                        v[ios!(M2S5FSRBModalF)] *= *fluid_damping_gain;
                    };
                    y
                }
                Six(ctrl) => {
                    let mut y = ctrl.outputs();
                    if let Some(ref mut v) = y {
                        v[ios!(M2S6FSCPModalF)] *= *modal_forces_gain;
                        v[ios!(M2S6FSRBModalF)] *= *fluid_damping_gain;
                    };
                    y
                }
                Seven(ctrl) => {
                    let mut y = ctrl.outputs();
                    if let Some(ref mut v) = y {
                        v[ios!(M2S7FSCPModalF)] *= *modal_forces_gain;
                        v[ios!(M2S7FSRBModalF)] *= *fluid_damping_gain;
                    };
                    y
                }
            })
            .collect::<Option<Vec<Vec<IO<Vec<f64>>>>>>()
            .map(|x| x.into_iter().flatten().collect())
    }

    fn inputs(&mut self, data: Option<Vec<IO<Self::Input>>>) -> Result<&mut Self, DOSIOSError> {
        use AsmController::*;
        if let Some(data) = data {
            self.controllers
                .iter_mut()
                .map(|controller| match controller {
                    One(ctrl) => ctrl
                        .inputs(Some(vec![
                            data[ios!(M2S1Cmd)].clone(),
                            data[ios!(M2S1FSRBModalD)].clone(),
                        ]))
                        .map(|_| ()),
                    Two(ctrl) => ctrl
                        .inputs(Some(vec![
                            data[ios!(M2S2Cmd)].clone(),
                            data[ios!(M2S2FSRBModalD)].clone(),
                        ]))
                        .map(|_| ()),
                    Three(ctrl) => ctrl
                        .inputs(Some(vec![
                            data[ios!(M2S3Cmd)].clone(),
                            data[ios!(M2S3FSRBModalD)].clone(),
                        ]))
                        .map(|_| ()),
                    Four(ctrl) => ctrl
                        .inputs(Some(vec![
                            data[ios!(M2S4Cmd)].clone(),
                            data[ios!(M2S4FSRBModalD)].clone(),
                        ]))
                        .map(|_| ()),
                    Five(ctrl) => ctrl
                        .inputs(Some(vec![
                            data[ios!(M2S5Cmd)].clone(),
                            data[ios!(M2S5FSRBModalD)].clone(),
                        ]))
                        .map(|_| ()),

                    Six(ctrl) => ctrl
                        .inputs(Some(vec![
                            data[ios!(M2S6Cmd)].clone(),
                            data[ios!(M2S6FSRBModalD)].clone(),
                        ]))
                        .map(|_| ()),
                    Seven(ctrl) => ctrl
                        .inputs(Some(vec![
                            data[ios!(M2S7Cmd)].clone(),
                            data[ios!(M2S7FSRBModalD)].clone(),
                        ]))
                        .map(|_| ()),
                })
                .collect::<Result<Vec<()>, DOSIOSError>>()
                .map(|_| self)
        } else {
            Err(DOSIOSError::Inputs(
                "Empty inputs passed to ASMS controllers".into(),
            ))
        }
    }
}
impl<'a> Iterator for ASMS<'a> {
    type Item = ();

    fn next(&mut self) -> Option<Self::Item> {
        use AsmController::*;
        self.controllers
            .iter_mut()
            .map(|controller| match controller {
                One(ctrl) => ctrl.step().map(|_| ()),
                Two(ctrl) => ctrl.step().map(|_| ()),
                Three(ctrl) => ctrl.step().map(|_| ()),
                Four(ctrl) => ctrl.step().map(|_| ()),
                Five(ctrl) => ctrl.step().map(|_| ()),
                Six(ctrl) => ctrl.step().map(|_| ()),
                Seven(ctrl) => ctrl.step().map(|_| ()),
            })
            .collect::<Result<Vec<()>, DOSIOSError>>()
            .ok()
            .map(|_| ())
    }
}
