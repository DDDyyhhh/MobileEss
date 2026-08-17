# Fixed Map Assets

This directory stores versioned metadata for fixed parking-lot maps. Large PCD
files remain local deployment assets and must not be committed to Git.

Create one directory for each capture, for example:

```text
docs/maps/parking_lot_2026-08-10_v1/
  metadata.yaml
```

Store the corresponding PCD under a deployment path such as:

```text
/var/lib/mower/maps/parking_lot_2026-08-10_v1/map.pcd
```

Do not overwrite a previously accepted PCD. Capture a new version instead.
