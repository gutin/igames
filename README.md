Interdiction games implementation
=================================

Based on http://www.optimization-online.org/DB_HTML/2013/04/3830.html

Contents:
This project consists of the following applications

- "main" which executes the main dynamic algorithms in the paper.
- "heurisitc" which runs the static stochastic and deterministic algorithms. Has a dependency on the CPLEX library.
- "psample" one of the experiments in the paper.. it works out interdiction probabilities based off a given 'policy file' (a file that encodes the decisions to be taken in each state).
- "unit-tests" - self-explanatory.

Installation:
Note: only targets unix atm ...
Also a little hacky but should work out for you. 

- You need to create a directory caleld 'install' which should be a sibling directory of 'src'. You can override this default install directory with the Make variable INSTALL_DIR.
- make -C src
- Installed binaries and static libraries (if all goes well) should end up in install directory

Any questions? Email gutineli@gmail.com.

Thx
