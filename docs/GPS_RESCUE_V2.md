# GPS Rescue V2

ESP-FC GPS Rescue V2 adds a home-relative NED navigation layer above the existing Angle V2, AltHold V2 and LAND V2 controllers.

Sensors -> GPS filtering / AltHold V2 -> local NED navigation -> GPS Rescue state machine -> horizontal roll/pitch targets + vertical-rate target -> existing attitude / AltHold V2 -> mixer.

Rescue never writes motor outputs directly and never creates a second altitude estimator. GPS/estimator loss requests the existing LAND V2 path when the vertical estimator remains healthy; otherwise the existing failsafe disarm path is used.

The failsafe option FAILSAFE_PROCEDURE_GPS_RESCUE keeps pilot input blocked while the Actuator supervisor activates Rescue only when GPS, attitude and altitude health are acceptable.

MSP2_ESPFC_GPS_RESCUE and MSP2_ESPFC_GPS_RESCUE_CONFIG expose runtime state and tuning to the matching Configurator.
