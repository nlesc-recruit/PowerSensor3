# Protocol
2 bytes
## first byte
3 bits for sensor id
last  bits for level

| 0 | 1 | 0 | 0 | 0 | 1 | 0 | 1 | 1 | 1 | 1 | 1 | 0 | 1 | 0 | 1 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| A | A | A | B | B | B | B | B | C | C | C | B | B | B | B | B |

| A | SensorID |
|---|----------|
| B | Level    |
|---|----------|
| C | Filler   |


## sensor types
| SensorType | Value |
|---|----------|
| ACS712-5 | .185    |
|---|----------|
| ACS712-20 | .1   |
|---|----------|
| ACS712-30 | .66   |
