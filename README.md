# kinetics: C++ library for Control, and Planning.


C++ library implementation of [_Modern Robotics: Mechanics, Planning, and Control_](https://modernrobotics.org) (Kevin Lynch and Frank Park, Cambridge University Press 2017).

functional implementation is available in Python, Matlab, Mathematica: [ Modern Robotics ](https://github.com/NxRLab/ModernRobotics/)

# required libraries:
- make sure to install eigens, igl libraries
- install algebra library see the instructions[Algebra](http://github.com/ertosns/algebra.git)


# installation

```console
user@name:~$ . ./install.sh

```

# verify kinetics is working by running example examples/exercises.cpp
```console
user@name:~/kinetics$ cd examples
user@name:~/kinetics/examples$ g++ exercises.cpp  -I /usr/include/eigen3 -lpthread -lalgebra -lkinetics && ./a.out
```
