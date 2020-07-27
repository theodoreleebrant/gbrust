// This file defines default trait for all MBCs and RamInfo, RomInfo
pub mod mbc_properties;
pub mod rom_only;
pub mod mbc1;
pub mod mbc2;
pub mod mbc3;
//mod mbc5;

pub use self::mbc_properties::*;
pub use self::rom_only::*;
pub use self::mbc1::*;
pub use self::mbc2::*;
pub use self::mbc3::*;
// pub use self::mbc5::*;
