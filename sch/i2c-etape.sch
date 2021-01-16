v 20130925 2
T 44000 40900 9 10 1 0 0 0 1
I2C eTape Interface
T 43800 40600 9 10 1 0 0 0 1
i2c-etape.sch
T 44000 40300 9 10 1 0 0 0 1
1
T 45500 40300 9 10 1 0 0 0 1
1
T 47800 40300 9 10 1 0 0 0 1
Michael Petersen
C 40000 40000 0 0 0 title-bordered-A.sym
N 42200 45800 42200 47200 4
C 42100 44300 1 0 0 gnd-1.sym
C 42000 45800 1 270 0 capacitor-1.sym
{
T 42700 45600 5 10 0 1 270 0 1
device=CAPACITOR
T 42300 45500 5 10 1 1 0 0 1
refdes=C1
T 42900 45600 5 10 0 0 270 0 1
symversion=0.1
T 42300 45200 5 10 1 1 0 2 1
value=1uF
T 42000 45800 5 10 0 0 0 0 1
footprint=0805
T 42300 45000 5 10 1 1 0 2 1
description=16V
}
C 49700 45800 1 0 0 header4-1.sym
{
T 50700 46450 5 10 0 0 0 0 1
device=HEADER3
T 50100 47500 5 10 1 1 0 0 1
refdes=J2
T 49700 45800 5 10 0 0 0 0 1
footprint=SMH-104-02-X-S
}
N 41900 47200 49700 47200 4
N 49700 46000 49200 46000 4
N 49200 44300 49200 46400 4
N 49200 46400 49700 46400 4
C 49400 45200 1 0 0 gnd-1.sym
N 49500 45500 49500 46800 4
N 49500 46800 49700 46800 4
T 50200 45700 9 10 1 0 0 5 1
eTape
C 44500 44600 1 0 1 ATtiny85-1.sym
{
T 43200 46050 5 10 1 1 0 6 1
refdes=U1
T 44225 47250 5 10 0 0 0 6 1
footprint=ATMEL_8S2
T 43600 46050 5 10 1 1 0 0 1
device=ATtiny85
}
C 41000 46000 1 0 0 qwiic-1.sym
{
T 42000 46650 5 10 0 0 0 0 1
device=QWIIC
T 41300 47850 5 10 1 1 0 3 1
refdes=J1
T 41800 48200 5 10 0 1 0 0 1
footprint=SM04B-SRSS
}
N 42200 44600 42200 44900 4
N 42700 45800 42200 45800 4
N 42700 44800 42200 44800 4
N 44500 45800 45900 45800 4
N 44800 45800 44800 46800 4
N 44800 46800 41900 46800 4
N 44500 45400 45400 45400 4
N 45100 45400 45100 46400 4
N 45100 46400 41900 46400 4
C 47300 45200 1 0 1 avrprog-1.sym
{
T 47300 46800 5 10 0 1 0 6 1
device=AVRPROG
T 47300 45200 5 10 0 0 0 6 1
footprint=TC2030-NL
T 46700 46500 5 10 1 1 0 6 1
refdes=J3
}
C 45600 45000 1 0 0 gnd-1.sym
N 45900 46200 45700 46200 4
N 45400 45600 45400 46800 4
N 45400 46800 48500 46800 4
N 47600 46800 47600 46200 4
N 47600 46200 47300 46200 4
N 45400 44900 45400 45400 4
N 45400 44900 47900 44900 4
N 47900 44900 47900 45800 4
N 47900 45800 47300 45800 4
N 45900 45400 45700 45400 4
N 45700 45400 45700 45300 4
N 47300 45400 47600 45400 4
N 47600 45400 47600 44600 4
N 47600 44600 45200 44600 4
N 45200 44600 45200 44800 4
N 45200 44800 44500 44800 4
N 44500 45600 45400 45600 4
N 45700 46200 45700 47200 4
N 49200 44300 45000 44300 4
N 45000 44300 45000 45200 4
N 45000 45200 44500 45200 4
T 41000 42000 9 10 1 0 0 0 5
32" eTape:
Vmin = 400ohm / 5400ohm = 0.074*Vref
Vmax = 5kohm / 10kohm = 0.5*Vref
(0.5 - 0.074) * 1024 = 436 counts
32" / 436 counts = 0.073" resolution
T 45500 42000 9 10 1 0 0 0 5
8" eTape:
Vmin = 400ohm / 1900ohm = 0.21*Vref
Vmax = 1.5kohm / 3kohm = 0.5*Vref
(0.5 - 0.21) * 1024 = 297 counts
8" / 297 counts = 0.027" resolution
C 42600 47500 1 90 0 gnd-1.sym
N 42300 47600 41900 47600 4
N 44500 45000 44700 45000 4
N 44700 43800 44700 45000 4
N 45000 43800 44700 43800 4
C 46200 43300 1 0 0 gnd-1.sym
N 46300 43600 46300 43800 4
N 46300 43800 46000 43800 4
C 48700 44600 1 90 0 led-3.sym
{
T 48700 44600 5 10 0 0 0 0 1
footprint=0805
T 48800 45050 5 10 1 1 90 5 1
device=RED
T 48400 45300 5 10 1 1 0 6 1
refdes=D1
}
C 48400 43700 1 0 0 gnd-1.sym
N 48500 44000 48500 44600 4
C 48600 45700 1 90 0 resistor-1.sym
{
T 48200 46000 5 10 0 0 90 0 1
device=RESISTOR
T 48600 45700 5 10 0 0 90 0 1
footprint=0805
T 48650 46250 5 10 1 1 0 1 1
refdes=R1
T 48650 46050 5 10 1 1 0 1 1
value=1k
}
N 48500 46600 48500 46800 4
N 48500 45700 48500 45500 4
C 45000 43800 1 0 0 switch-pushbutton-no-1.sym
{
T 45500 43700 5 10 1 1 0 5 1
refdes=SW1
T 45400 44400 5 10 0 0 0 0 1
device=SWITCH_PUSHBUTTON_NO
T 45000 43800 5 10 0 0 0 0 1
footprint=CK_KXT3
}
