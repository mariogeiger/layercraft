//! What a bead is for, as three independent facts: its kind (a wall's loop, a dense line or a
//! sparse one), the face of the part it makes (the side, the top, the bottom, or none: inside),
//! and what it rests on (solid plastic, sparse infill, or air).
//!
//! Each consumer reads only the fact it needs. A dense line resting on anything but solid is a
//! strand, round in section; what shows goes slower; what lies on air bridges. The features
//! slicers name, Bambu Studio's among them, are combinations of the three: `name` writes one,
//! `from_name` reads one back.

use serde::{Deserialize, Serialize};

/// What a bead is: a loop of the walls, or a line of infill, dense or sparse.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Kind {
    Wall,
    Solid,
    Sparse,
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

/// What a bead is laid on.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Support {
    Solid,
    Sparse,
    Air,
}

/// What a bead is for.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct Role {
    pub kind: Kind,
    pub face: Face,
    pub support: Support,
}

const fn role(kind: Kind, face: Face, support: Support) -> Role {
    Role { kind, face, support }
}

impl Role {
    pub const OUTER_WALL: Role = role(Kind::Wall, Face::Outside, Support::Solid);
    pub const OVERHANG_WALL: Role = role(Kind::Wall, Face::Outside, Support::Air);
    pub const INNER_WALL: Role = role(Kind::Wall, Face::Inside, Support::Solid);
    pub const FLOATING_WALL: Role = role(Kind::Wall, Face::Inside, Support::Sparse);
    pub const TOP_SURFACE: Role = role(Kind::Solid, Face::Top, Support::Solid);
    pub const BOTTOM_SURFACE: Role = role(Kind::Solid, Face::Bottom, Support::Solid);
    pub const SOLID: Role = role(Kind::Solid, Face::Inside, Support::Solid);
    pub const BRIDGE: Role = role(Kind::Solid, Face::Bottom, Support::Air);
    pub const INTERNAL_BRIDGE: Role = role(Kind::Solid, Face::Inside, Support::Sparse);
    pub const SPARSE: Role = role(Kind::Sparse, Face::Inside, Support::Sparse);

    /// The roles slicers name, in a fixed order: the codes of a legend.
    pub const NAMED: [Role; 10] = [
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
    ];

    /// Whether the bead is a strand, round in section: a dense line on anything but solid.
    pub fn strand(self) -> bool {
        self.kind == Kind::Solid && self.support != Support::Solid
    }

    /// The feature name slicers write for this role.
    pub fn name(self) -> &'static str {
        let shows = self.face != Face::Inside;
        match (self.kind, self.support) {
            (Kind::Wall, Support::Solid) if shows => "Outer wall",
            (Kind::Wall, Support::Solid) => "Inner wall",
            (Kind::Wall, _) if shows => "Overhang wall",
            (Kind::Wall, _) => "Floating vertical shell",
            (Kind::Solid, Support::Air) => "Bridge",
            (Kind::Solid, Support::Sparse) => "Internal Bridge",
            (Kind::Solid, Support::Solid) => match self.face {
                Face::Top => "Top surface",
                Face::Bottom => "Bottom surface",
                Face::Outside | Face::Inside => "Internal solid infill",
            },
            (Kind::Sparse, _) => "Sparse infill",
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
