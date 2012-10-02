AESTE LLVM Port
=============== 

This is a port of the upstream Low-Level Virtual Machine (LLVM)
compiler to AESTE platforms.

The purpose of maintaining this port is as a record/archive of the
upstream code. It will also be our main repository for any
customisations made to the upstream code.

Historical Summary
------------------
* 12.35: T3RAS alpha release.
* 12.27: LLVM-3.1 release.

Code Organisation
-----------------
This repository is organised using the structure proposed at nvie.com
with two main public branches:

* master - holds the tagged public release code. Track this branch if
  you only wish to receive hotfixes and work on stable code.

* develop - holds the active development branch. Track this branch if
  you wish to stay updated with all the latest developments and work
  on unstable code.

If you wish to contribute to the development of this port, please fork
this repository, make your changes and perform a pull request.

Upstream Repository 
------------------- 
This repository is a fork of the upstream GIT repository hosted at
http://llvm.org/git/llvm.git

Reporting Bugs
--------------
Any bugs spotted in the upstream code should be reported to the
upstream source. Any bugs for our custom code should be reported using
the issues page at the github repository.
