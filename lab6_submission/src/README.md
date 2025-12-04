# README.md : Lab 6 Submission

The `Makefile` contains a set of commands for building and running the python version with make. 

## Specifying Test Files
Enter the name of the test file by overriding arguments or enter the full path to the file
Defaults are `START=1, END=5, NODES_FILE=nodes_feasible.csv EDGES_FILE=edges_feasible.csv PRIORITIES_FILE=priorities.csv DATA_DIR=../tests`
```bash
make run NODES_FILE=nodes_feasible.csv EDGES_FILE=edges_feasible.csv PRIORITIES_FILE=priorities.csv
make run NODES=../tests/nodes_feasible.csv EDGES=../tests/nodes_feasible.csv PRIORITIES=../tests/priorities.csv
```

### Override arguments as needed, e.g.:
```bash
make run START=5 END=17
make run NODES=data/a.csv EDGES=data/b.csv
```