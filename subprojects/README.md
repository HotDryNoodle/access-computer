# Meson subprojects

All third-party libraries are fetched and built from this directory.

| Dependency      | Wrap file              | Source tree              |
|-----------------|------------------------|--------------------------|
| nlohmann_json   | `nlohmann_json.wrap`   | `nlohmann_json-3.12.0/`  |
| libzmq          | `libzmq.wrap`          | `libzmq-4.3.5/`          |

`libzmq` uses a Meson overlay in `packagefiles/libzmq/` (pure Meson build + Linux `platform.hpp`).

## Bootstrap

```bash
cd ~/projects/mission-planer
meson subprojects download
meson setup build -Dzmq=enabled
meson compile -C build
```

`packagecache/` stores downloaded tarballs and can be regenerated.
