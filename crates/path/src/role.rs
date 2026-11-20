//! What a bead is for, as three independent facts: its kind (a wall's loop, a dense line, a
//! sparse one, or a line of the supports, which hold the part up and come off it), the face it
//! makes (the side, the top, the bottom, or none: inside; a support's top is the face the part
//! rests on), and what it rests on (solid plastic, sparse infill, or air).
//!
//! Each consumer reads only the fact it needs. A dense line resting on anything but solid is a
//! strand, round in section; what shows goes slower; what lies on air bridges. The features
//! slicers name, Bambu Studio's among them, are combinations of the three: `name` writes one,
//! `from_name` reads one back.

use serde::{Deserialize, Serialize};

/// What a bead is: a loop of the walls, a line of infill, dense or sparse, or a line of the
/// supports under the part.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Kind {
    Wall,
    Solid,
    Sparse,
    Support,
}

/// The face of the part a bead makes: the side, the top, the bottom, or none.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Face {
    Outside,
    Top,
    Bottom,
    Inside,
}

/// What a bead rests on.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Rest {
    Solid,
    Sparse,
    Air,
}

/// What a bead is for.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct Role {
    pub kind: Kind,
    pub face: Face,
    #[serde(alias = "support")]
    pub rest: Rest,
}

const fn role(kind: Kind, face: Face, rest: Rest) -> Role {
    Role { kind, face, rest }
}

impl Role {
    pub const OUTER_WALL: Role = role(Kind::Wall, Face::Outside, Rest::Solid);
    pub const OVERHANG_WALL: Role = role(Kind::Wall, Face::Outside, Rest::Air);
    pub const INNER_WALL: Role = role(Kind::Wall, Face::Inside, Rest::Solid);
    pub const FLOATING_WALL: Role = role(Kind::Wall, Face::Inside, Rest::Sparse);
    pub const TOP_SURFACE: Role = role(Kind::Solid, Face::Top, Rest::Solid);
    pub const BOTTOM_SURFACE: Role = role(Kind::Solid, Face::Bottom, Rest::Solid);
    pub const SOLID: Role = role(Kind::Solid, Face::Inside, Rest::Solid);
    pub const BRIDGE: Role = role(Kind::Solid, Face::Bottom, Rest::Air);
    pub const INTERNAL_BRIDGE: Role = role(Kind::Solid, Face::Inside, Rest::Sparse);
    pub const SPARSE: Role = role(Kind::Sparse, Face::Inside, Rest::Sparse);
    /// A support's line, standing on the one below it.
    pub const SUPPORT: Role = role(Kind::Support, Face::Inside, Rest::Solid);
    /// A support's top, what the part rests on: its first layer, spanning the column over air
    /// from its loop to its loop.
    pub const SUPPORT_INTERFACE: Role = role(Kind::Support, Face::Top, Rest::Air);
    /// The layers of a support's top over its first, each resting on the one below.
    pub const SUPPORT_INTERFACE_RESTING: Role = role(Kind::Support, Face::Top, Rest::Solid);

    /// The roles slicers name, in a fixed order: the codes of a legend.
    pub const NAMED: [Role; 12] = [
        Role::OUTER_WALL,
        Role::OVERHANG_WALL,
        Role::INNER_WALL,
        Role::FLOATING_WALL,
        Role::TOP_SURFACE,
        Role::BOTTOM_SURFACE,
        Role::SOLID,
        Role::BRIDGE,
        Role::INTERNAL_BRIDGE,
        Role::SPARSE,
        Role::SUPPORT,
        Role::SUPPORT_INTERFACE,
    ];

    /// The role of a dense line inside the part on layer `index`: the first layer's make the
    /// part's bottom.
    pub fn solid_on(index: usize) -> Role {
        if index == 0 { Role::BOTTOM_SURFACE } else { Role::SOLID }
    }

    /// Whether the bead is a strand, round in section: a dense line on anything but solid.
    pub fn strand(self) -> bool {
        self.kind == Kind::Solid && self.rest != Rest::Solid
    }

    /// The feature name slicers write for this role.
    pub fn name(self) -> &'static str {
        let shows = self.face != Face::Inside;
        match (self.kind, self.rest) {
            (Kind::Wall, Rest::Solid) if shows => "Outer wall",
            (Kind::Wall, Rest::Solid) => "Inner wall",
            (Kind::Wall, _) if shows => "Overhang wall",
            (Kind::Wall, _) => "Floating vertical shell",
            (Kind::Solid, Rest::Air) => "Bridge",
            (Kind::Solid, Rest::Sparse) => "Internal Bridge",
            (Kind::Solid, Rest::Solid) => match self.face {
                Face::Top => "Top surface",
                Face::Bottom => "Bottom surface",
                Face::Outside | Face::Inside => "Internal solid infill",
            },
            (Kind::Sparse, _) => "Sparse infill",
            (Kind::Support, _) if self.face == Face::Top => "Support interface",
            (Kind::Support, _) => "Support",
        }
    }

    /// The role a slicer's feature name stands for; gap infill is solid infill, narrower.
    pub fn from_name(name: &str) -> Option<Role> {
        match name {
            "Gap infill" => Some(Role::SOLID),
            _ => Role::NAMED.into_iter().find(|r| r.name() == name),
        }
    }

    /// The role's code in `NAMED`, the same in every stage's picture.
    pub fn code(self) -> usize {
        Role::NAMED.iter().position(|r| r.name() == self.name()).unwrap_or(0)
    }
}
