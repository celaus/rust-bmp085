
// Copyright 2019 Claus Matzinger
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


use std::error::Error;
// Copy of the traits that used to come with the i2cdev library but are now gone ...


/// Trait for sensors that provide access to temperature readings
pub trait Thermometer {
    type Error: Error;

    /// Get a temperature from the sensor in degrees celsius
    ///
    /// Returns `Ok(temperature)` if available, otherwise returns
    /// `Err(Self::Error)`
    fn temperature_celsius(&mut self) -> Result<f32, Self::Error>;
}

/// Trait for sensors that provide access to pressure readings
pub trait Barometer {
    type Error: Error;

    /// Get a pressure reading from the sensor in kPa
    ///
    /// Returns `Ok(temperature)` if avialable, otherwise returns
    /// `Err(Self::Error)`
    fn pressure_kpa(&mut self) -> Result<f32, Self::Error>;
}
