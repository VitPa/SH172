# Legenda per le variabili utilizzate nel simulatore

## CONDIZIONI INIZIALI - [CI]

## ENGINE - [engine]

| Indice | Descrizione                                          | Unità     |
|--------|------------------------------------------------------|-----------|
| 0      | Potenza massima sl                                   | [kW]      |
| 1      | Legge variazionale della potenza con la quota        | [-]       |
| 2      | Numero di giri motore minimo                         | [rpm]     |
| 3      | Numero di giri motore massimo                        | [rpm]     |
| 4      | Rendimento meccanico trasmissione                    | [-]       |
| 5      | Consumo specifico                                    | [kg/s]    |


## PROPELLER
### [geometry_propeller]

| Indice | Descrizione          | Unità     |
|--------|----------------------|-----------|
| 0      | Diametro             | [m]       |
| 1      | Diametro ogiva       | [m]       |
| 2      | Numero di pale       | [-]       |
| 3      | Inerzia              | [kg·m²]   |
| 4      | Numero di stazioni   | [-]       |


### [propeller_profile]

| Indice | Descrizione   | Unità       |
|--------|----------------|-------------|
| 0      | Clalfa         | [rad⁻¹]     |
| 1      | Cl0            | [-]         |
| 2      | a0             | [rad]       |
| 3      | Cdalfa2        | [rad⁻²]     |
| 4      | Cdalfa         | [rad⁻¹]     |
| 5      | Cd0            | [-]         |

### [data_propeller]

| Indice | Descrizione  | Unità    |
|--------|--------------|----------|
| x,0    | CSI          | [-]      |
| x,1    | RD           | [m]      |
| x,2    | CH AD        | [-]      |
| x,3    | BA           | [deg]    |

## DBA
### [body_axes]

| Indice | Descrizione                                                                  | Unità     |
|--------|------------------------------------------------------------------------------|-----------|
| 0      | Massa                                                                        | [kg]      |
| 1      | Apertura alare                                                               | [m]       |
| 2      | Superficie alare                                                             | [m²]      |
| 3      | Corda                                                                        | [m]       |
| 4      | Mach drag rise                                                               | [-]       |
| 5      | Distanza asse spinta - baricentro X lungo Z                                  | [m]       |
| 6      | Distanza asse spinta - baricentro X lungo Y                                  | [m]       |
| 7      | Distanza asse spinta - baricentro X lungo Z                                  | [m]       |
| 8      | Angolo asse spinta vs asse X nel piano X-Y (dtz) (>0 verso DX)               | [deg]     |
| 9      | Angolo asse spinta vs asse X nel piano X-Z (dtY) (>0 verso l'alto)           | [deg]     |
| 10     | Numero di incidenze disponibili                                              | [-]       |
| 11     | Rotary derivatives presenti (1 --> sì)                                       | [-]       |
| 12     | Posizione di riferimento baricentro                                          | [m]       |
| 13     | Jx                                                                           | [kg·m²]   |
| 14     | Jy                                                                           | [kg·m²]   |
| 15     | Jz                                                                           | [kg·m²]   |
| 16     | Jxz                                                                          | [kg·m²]   |
| 17     | Opzione posizione baricentro                                                 | [-]       |
| 18     | Nuova posizione del baricentro                                               | [m]       |
| 19     | Posizione pilota lungo X (rispetto al C.G.)                                  | [m]       |
| 20     | Posizione pilota lungo Y (rispetto al C.G.)                                  | [m]       |
| 21     | Posizione pilota lungo Z (rispetto al C.G.)                                  | [m]       |

### [deflection_limits]

| Indice | Descrizione                   | Unità  |
|--------|-------------------------------|--------|
| 0      | Elevator (max)                | [deg]  |
| 1      | Elevator (min)                | [deg]  |
| 2      | Ailerons (symmetrical)        | [deg]  |
| 3      | Rudder (symmetrical)          | [deg]  |
| 4      | Flap (min deflection)         | [deg]  |
| 5      | Flap (max deflection)         | [deg]  |

### [fuel_mass]

| Indice | Descrizione                               | Unità |
|--------|-------------------------------------------|-------|
| 0      | Opzione massa (0: costante, 1: variabile) | [-]   |
| 1      | Frazione di massa (% MTOW)                | [-]   |

### [steady_state_coefficients]

| Indice | Descrizione | Unità |
|--------|-------------|--------|
| x,0    | ALPHA       | [-]    |
| x,1    | CX          | [-]    |
| x,2    | CY          | [-]    |
| x,3    | CZ          | [-]    |
| x,4    | Cl          | [-]    |
| x,5    | Cm          | [-]    |
| x,6    | Cn          | [-]    |

### aerodynamic_derivatives
#### x force derivatives - [aer_der_x]

| Indice | Descrizione | Unità |
|--------|-------------|--------|
| x,0    | ALPHA       | [-]    |
| x,1    | CXA         | [-]    |
| x,2    | CXAP        | [-]    |
| x,3    | CXU         | [-]    |
| x,4    | CXQ         | [-]    |
| x,5    | CXB         | [-]    |
| x,6    | CXP         | [-]    |
| x,7    | CXR         | [-]    |

#### y force derivatives - [aer_der_y]

| Indice | Descrizione | Unità |
|--------|-------------|--------|
| x,0    | ALPHA       | [-]    |
| x,1    | CYB         | [-]    |
| x,2    | CYBP        | [-]    |
| x,3    | CYP         | [-]    |
| x,4    | CYR         | [-]    |
| x,5    | CYA         | [-]    |
| x,6    | CYQ         | [-]    |

#### z force derivatives - [aer_der_z]

| Indice | Descrizione | Unità |
|--------|-------------|--------|
| x,0    | ALPHA       | [-]    |
| x,1    | CZALPHA     | [-]    |
| x,2    | CZAP        | [-]    |
| x,3    | CZU         | [-]    |
| x,4    | CZQ         | [-]    |
| x,5    | CZB         | [-]    |
| x,6    | CZP         | [-]    |
| x,7    | CZR         | [-]    |

#### [rolling_moment_der]

| Indice | Descrizione | Unità |
|--------|-------------|--------|
| x,0    | ALPHA       | [-]    |
| x,1    | ClB         | [-]    |
| x,2    | ClBP        | [-]    |
| x,3    | ClP         | [-]    |
| x,4    | ClR         | [-]    |
| x,5    | ClA         | [-]    |
| x,6    | ClQ         | [-]    |

#### [pitch_moment_der]

| Indice | Descrizione | Unità |
|--------|-------------|--------|
| x,0    | ALPHA       | [-]    |
| x,1    | CmA         | [-]    |
| x,2    | CmAP        | [-]    |
| x,3    | CmU         | [-]    |
| x,4    | CmQ         | [-]    |
| x,5    | CmB         | [-]    |
| x,6    | CmP         | [-]    |
| x,7    | CmR         | [-]    |

#### [yawing_moment_der]

| Indice | Descrizione | Unità |
|--------|-------------|--------|
| x,0    | ALPHA       | [-]    |
| x,1    | CnB         | [-]    |
| x,2    | CnBP        | [-]    |
| x,3    | CnP         | [-]    |
| x,4    | CnR         | [-]    |
| x,5    | CnA         | [-]    |
| x,6    | CnQ         | [-]    |

#### [control_force_der]

| Indice | Descrizione | Unità |
|--------|-------------|--------|
| x,0    | ALPHA       | [-]    |
| x,1    | CXde        | [-]    |
| x,2    | CXdle       | [-]    |
| x,3    | CZde        | [-]    |
| x,4    | CZdle       | [-]    |
| x,5    | CYda        | [-]    |
| x,6    | CYdr        | [-]    |

#### [control_moment_der]

| Indice | Descrizione | Unità |
|--------|-------------|--------|
| x,0    | ALPHA       | [-]    |
| x,1    | Clda        | [-]    |
| x,2    | Cldr        | [-]    |
| x,3    | Cmde        | [-]    |
| x,4    | Cmdle       | [-]    |
| x,5    | Cnda        | [-]    |
| x,6    | Cndr        | [-]    |

#### [rotary_der]

| Indice | Descrizione | Unità |
|--------|-------------|--------|
| x,0    | ALPHA       | [-]    |
| x,1    | CXom        | [-]    |
| x,2    | CYom        | [-]    |
| x,3    | CZom        | [-]    |
| x,4    | Clom        | [-]    |
| x,5    | Cmom        | [-]    |
| x,6    | Cnom        | [-]    |