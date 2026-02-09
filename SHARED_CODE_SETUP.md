# Instructions for Using Shared Code with Real PROS Project

The real PROS project in `PROS_TEST/` should be configured to use the shared robot code.

## Option 1: Symlinks (Recommended for development)

On Windows (requires admin or Developer Mode):
```powershell
cd PROS_TEST
Remove-Item src\main.cpp
Remove-Item include\main.h
New-Item -ItemType SymbolicLink -Path src\main.cpp -Target ..\shared\src\main.cpp
New-Item -ItemType SymbolicLink -Path include\main.h -Target ..\shared\include\main.h
```

On Linux/Mac:
```bash
cd PROS_TEST
rm src/main.cpp include/main.h
ln -s ../shared/src/main.cpp src/main.cpp
ln -s ../shared/include/main.h include/main.h
```

## Option 2: Copy Files (Simpler but requires manual sync)

```bash
cp shared/src/main.cpp PROS_TEST/src/main.cpp
cp shared/include/main.h PROS_TEST/include/main.h
```

After making changes to shared code, copy again.

## Building Real PROS Project

```bash
cd PROS_TEST
pros make
```

## Why Shared Code?

- Write robot logic once
- Test in simulator before uploading to robot
- Faster iteration (no compilation for ARM, no upload time)
- Same behavior guaranteed on simulator and real hardware
