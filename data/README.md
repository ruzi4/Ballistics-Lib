# data/

Reference data files used at runtime by the library examples and renderer.

## `munitions.json`

A JSON array of projectile specifications. Each entry is a `MunitionSpec` and
can be loaded with `MunitionLibrary::load()`.

### Schema

```json
{
  "name":                "string  — unique identifier",
  "mass_kg":             "number  — projectile mass (kg)",
  "density_kg_m3":       "number  — material density (kg/m³)",
  "reference_area_m2":   "number  — cross-sectional reference area (m²)",
  "drag_coefficient":    "number  — dimensionless Cd",
  "muzzle_velocity_ms":  "number  — muzzle velocity (m/s)",
  "diameter_m":          "number  — characteristic diameter (m), optional"
}
```

### Included munitions

| Name | Mass | Cd | Muzzle velocity | Diameter | Notes |
|---|---|---|---|---|---|
| `9mm_fmj_115gr` | 7.45 g | 0.295 | 370 m/s | 9.0 mm | Pistol / SMG |
| `5.56x45_m855_62gr` | 4.02 g | 0.307 | 930 m/s | 5.56 mm | NATO intermediate rifle |
| `7.62x51_m80_147gr` | 9.53 g | 0.295 | 853 m/s | 7.62 mm | NATO battle rifle / GPMG |
| `338_lapua_250gr` | 16.2 g | 0.230 | 905 m/s | 8.58 mm | Long-range precision |
| `50_bmg_660gr` | 42.8 g | 0.620 | 928 m/s | 12.7 mm | Heavy machine gun / anti-materiel |

### Adding new munitions

Append additional objects to the JSON array and pass the file path to
`MunitionLibrary::load()`. Multiple files can be loaded into the same library
instance; entries are merged by name.
