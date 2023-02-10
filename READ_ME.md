# IMA904 Project : XPBD, Position-based simulation of compliant constrained dynamics
### by Sammy Rasamimanana

In this project, we implemented an XPBD solver in `C++` with a basic renderer using `freeglut` and `glew`. Our implementation heavily relies on the  `Eigen` library to manipulate 3D vector and matrices. We used an oriented-object approach to manipulate the meshes and the constraints. Please refer to the `IMA904_report.pdf` file for more details. <br>

## Example of use
The roadmap for this project was to implement small scenes to play with the different constraints and their parameters. To see those scenes, go to the folder `./src`, and enter in the terminal the commands described in the next subsections.

### Spring
```
make spring
./spring
```
This will compile then execute the `spring.cpp` file which aims to display two particles linked by a distance constraint. One of this particle is fixed.

### Cloth
```
make cloth
./cloth
```
This will compile then execute the `demo_cloth.cpp` file which aimes to display a cloth with its top two vertices fixed.

### Wall
```
make wall
./wall
```
This will compile then execute the `wall.cpp` file which displays one bouncing ball

### Balls
```
make ball
./ball
```
This will compile then execute the `ball.cpp` file which displays a bunch of balls that are constrained to stay inside a box. The front face and the back face of the box are not displayed for clarity and visualisation purpose. User can add a ball to the scene by pressing `F1`.

### Main scene
```
make run
```
This will compile and execute `main.cpp` , the main scene of the project, which is basically a cloth balloon scene.

## Object class
We designed a generic class `Object` with
mass, position, speed and color attributes. Those attributes will be
used to set and render the object in the space, but are not going to
be updated. Instead, each object will have a `std::vector` of `Particle` which will store its vertices. Those
vertices are the ones that are going to see their position and speed
updated to move the object.
<br> Here are the different classes that inherits or are involved with the `Object` class: <br>

### Particle
A class implementing the particles which are going to be the core of the (X)PBD process.

### Sphere
A class representing a sphere. Basically it is a particule with a boundary radius.

### Cloth
A class representing a cloth, which is actually a rectangle of particles. This rectangle is subdivided in quads which are virtually subdivided in triangles.

### Plane
A class representing a finite plane in the 3D space. It is useful to place walls in the scene

### Floor
The floor of the scene.

## Constraints
The other important class is the `Constraint` which define the main attributes used to solve a constraint. Then, each subclass of constraints has its own attributes to do the calculation.

### Distance Constraint
This tries to maintain two particles at a certain distance. It gives a similar effect
than a spring between those two particles.

### Fixed Constraint
It makes a particle staying at a given position.

### Collision Constraint
It is basically the inequality version of the distance constraint, and allows us to avoid the collision between two particles.

### Penetration Constraint
This constraint prevents a vertex `q` from entering the triangle formed by particles `(p1,p2,p3)`.

### Wall Constraint
It is the particular case of the penetration constraint when  `(p1,p2,p3)` are not particles but static points of a given plane that acts like a wall or a floor.

### Bending Constraint
We constraint the two half-triangles to keep the same angle.

### Isobending Constraint
Two adjacent triangles should keep the same bending energy overtime.