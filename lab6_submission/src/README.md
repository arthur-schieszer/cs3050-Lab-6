# README.md : Lab 6 Submission

The `Makefile` contains a set of commands for building and running the python version with make. 

## Specifying Test Files
Enter the name of the test file by overriding arguments or enter the full path to the file
```bash
make run NODES_FILE=nodes_feasible.csv EDGES_FILE=edges_feasible.csv
make run NODES=../tests/nodes_feasible.csv EDGES=../tests/nodes_feasible.csv

### Override arguments as needed, e.g.:
```bash
make run START=5 END=17
make run NODES=data/a.csv EDGES=data/b.csv
```