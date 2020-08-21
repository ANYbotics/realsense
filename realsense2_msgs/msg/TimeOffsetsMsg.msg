# Time offsets.
float64         frame_acquisition_offset                # Delay introduced by ASIC processing frames.   <DEVICE>         (Unit: milliseconds)
float64         wire_transmission_offset                # Delay due to wire transmission.               <WIRE>           (Unit: milliseconds)
float64         driver_handover_offset                  # Delay due to kernel-user space transition.    <PC>             (Unit: milliseconds)