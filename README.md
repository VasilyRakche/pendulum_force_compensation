## MuJoCo example for Bias Forces Compensation

### Running the simulation
**Project Configured for LINUX system.**   
Place it in mujoco folder in `sample/` directory. 
Make sure [eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page) is installed on your system. 

To run:
```
./run_unix          # by default only builds the code
./bin/dbpendulum    # code executable 
```

Procedure is also automated for VS Code:
- Open the project from VS Code
- On `F5` it should build and run executable in debug mode (Configured as `Pusk`)
- In debug tab, change to `Pusk no build` to just run the executable without re-building

Assuming that the MuJoCo is [installed](https://mujoco.readthedocs.io/en/stable/programming/index.html?highlight=install#getting-started) and has the following structure:
```
- mujoco/
    - include/
    - bin/
    - lib/
    - share/
    - sample/
```