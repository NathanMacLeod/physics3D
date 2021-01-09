# physics3D
This project is a fairly simple rigid body 3d physics engine I am working on.

The main project is the physics engine, but I also have built a 3d rendering program to go along with it. The testing and 3d program is built on top of the
[Pixel Game Engine](https://github.com/OneLoneCoder/olcPixelGameEngine), which is a really easy to work program (only requires including one header file) that provides
a cross platform canvas for rendering on, and support for user input and sound. It by default only supports 2d functions, (although there are add-ons for 3d, and 
there is a really nice [series by the guy who made the engine on 3d rendering](https://www.youtube.com/watch?v=ih20l3pJoeU)).

However I just used the 2d library and programmed the 3d stuff myself. The 3d rendering uses a Z buffer in order to draw objects at the correct depth. 

Rendering with color based on Z buffer data:
![imgur](https://i.imgur.com/WTEMVGM.png)

In both the rendering and physics engine, orientation is represented using Rotors. In 3D rotors are equivalent to quaternions, but the math is conceptually a bit easier
as the concepts still exist in 3d. I found [this youtube series](https://www.youtube.com/watch?v=PNlgMPzj-7Q&list=PLpzmRsG7u_gqaTo_vEseQ7U8KFvtiJY4K)
very helpful for learning the geometric algebra to understand rotors.

# Physics implementation



The physics engine allows for the representation of rigid bodies, both concave and covnex. Concave rigid bodies are representing as the union of multiple pre-defined convex hulls. Currently im having issues with the concave bodies working properly, but the convex ones work pretty good.

Convex hulls represent a convex polyhedron, and a density. The Inertia tensor and center of mass of a convex hull is calculated using the method outline in
[Fast and Accurate Computation of Polyhedral Mass Properties by Brian Miritch](http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.56.127&rep=rep1&type=pdf).
The method outlined in that paper works for both convex and concave polyhedra, however I only use it for convex polyhedron.

On creation a rigid body calculates its inertia tensor and center of mass based on the inertia tensor and center of mass of the convex hulls that define it. The body tracks only
the inertia tensor relative to the body frame, and instead calculates vectors relative to the body frame when interacting with the inertia tensor.

Collision detection is done using the separating axis theorem. In 3d this requires using the normal of each face and the normals of each cross products edge, although I found
[these presentation slides](http://twvideo01.ubm-us.net/o1/vault/gdc2013/slides/822403Gregorius_Dirk_TheSeparatingAxisTest.pdf) by Dirk Gregorious that show a more optomized
algorthim that I plan on looking into implementing.

Contact resolution is probably the weakest part of the physics engine currently, objects in contact with the ground tend to jitter and there currently is not a a system
to turn off bodies that are at rest, which is a very significant optimization. The main reason these parts are still missing is because I am currently working on a project
using this physics engine set in a zero gravity enviornment, so these factors are less signifcant in this enviorment.

The engine uses an Octree data structure to optimize the number of narrow tests required. The octree recursivley partitions space, allowing comparing only objects sharing
the same space in the octree rather than n^2 narrow tests of every body against each other.

Drawing the octree partitions

![octreegif](https://i.imgur.com/SdLJ3p6.gif)

The engine also implements the gyroscopic evolution of angular velocity over time, which allows recreating phenomenon such as the [Dzhanibekov effect](https://youtu.be/f6z7WA7U7NA?t=19)

Dhzanibekov effect in engine

![dhzanibekov effect](https://i.imgur.com/til4v4W.gif)


If the equations for the gyroscopic equation are solved explicity in each time step, the system tends to spiral out of control and spin faster and faster. To implement
the gyroscopic equations I followed the method for implicit integration outlined by Errin Catto in [this presentation](https://www.gdcvault.com/play/1022196/Physics-for-Game-Programmers-Numerical)
(The part he talks more specifically about gyroscopic motion starts around 39:00)

# Demonstrations
https://imgur.com/a/finan52
