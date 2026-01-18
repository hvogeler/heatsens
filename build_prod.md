# Production Build Instructions

## Overview

This project uses a separate `sdkconfig.prod` file for production builds to maintain different settings from development builds.

## Creating the Production Config

1. Copy the current sdkconfig as a starting point:
   ```bash
   cp sdkconfig sdkconfig.prod
   ```

2. Edit production settings:
   ```bash
   idf.py -D SDKCONFIG=sdkconfig.prod menuconfig
   ```

3. Recommended production settings to change:
   - **Compiler options → Optimization Level**: Set to `-O2` (performance) or `-Os` (size)
   - **Component config → Log output → Default log verbosity**: Set to `Warning` or `Error`
   - **Compiler options → Assertion level**: Set to `Silent` or `Disabled`
   - **Application manager → Exclude time/date from app image**: Enable for reproducible builds

## Building for Production

```bash
idf.py -B build_prod -D SDKCONFIG=sdkconfig.prod build
```

This creates the production build in the `build_prod/` directory, keeping it separate from development builds.

## Flashing Production Firmware

```bash
idf.py -B build_prod -D SDKCONFIG=sdkconfig.prod flash
```

## Notes

- The `build_prod/` directory is separate from `build/`, so you can maintain both builds simultaneously
- Always test production builds before deployment since optimizations and disabled logging may affect behavior
- Add `sdkconfig.prod` to version control to ensure reproducible production builds
