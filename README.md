# Articulated Object Data Generation
## Generate Objects

Choose an object type and number of variants:

```bash
python3 generate.py \
  --object knife \
  --num 30 \
  --config configs/batch/knife.toml \
  --dir ../assets/objects/knife_30
```

Supported objects are `knife`, `lighter`, `stapler`, and `tong`.
Always pass the matching `configs/batch/<object>.toml` file for batch
generation. Edit that file to change size ranges, joint limits, or other
generation parameters.

The example above writes to `../assets/objects/knife_30/<id>/mobility.urdf`.
Each batch also writes `lbx.json`.

Batch config fields:

- Knife: `body_size_ranges`, `slider_size_ranges`,
  `edge_clearance_range`, `joint_travel`
- Lighter: `body_size_ranges`, `cap_height_range`, `joint_lower`,
  `joint_upper`
- Stapler: `link_size_ranges`, `joint_lower_range`, `joint_upper`
- Tong: `link_size_ranges`, `joint_lower_range`, `joint_upper`

Each size-range vector contains one `[min, max]` pair per `[x, y, z]` axis.
Values are stratified into `--num` bins and randomly regrouped across axes.


### Deterministic Digital Twins

Use a measurement config without `--num` to generate one exact object:

```bash
python3 generate.py \
  --object knife \
  --config configs/single/knife.toml \
  --dir ../assets/objects/knife_real
```

Deterministic generation always appends. If `000` already exists, the next run
creates `001`, then `002`, and so on. Existing objects are never rewritten, and
the new entry is merged into `lbx.json`.

Editable single-object configs are available in
[`configs/single`](configs/single).

## Label Objects

```bash
python3 label.py \
  --object knife \
  --dir ../assets/objects/knife_30
```
Each label script only defines its material rules and joint pose. Shared URDF
loading, transforms, OBJ/MTL writing, and directory traversal are implemented
in `object_tools/labeling.py`.

## Label Colors

- Green: thumb touchable
- Blue: fingers touchable
- Red: forbidden
- Gray: never sample (interior surface)
