#![no_std]

//! MAVLink 2 adapter. We depend on the upstream `mavlink` crate lazily from
//! the application binary so this crate stays a thin project-specific shim.
