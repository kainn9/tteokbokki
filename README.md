# tteokbokki(WIP)
A basic 2D physics library for Go, inspired by the [Pikuma C++ course](https://pikuma.com/courses/game-physics-engine-programming), and written in an ECS-based fashion.


### Check List
- Fix Joint constraint pattern(no pointers to components in components && use vec2 as anchor instead of a second RigidBody) []
- Clean up Phys package(transformer/resolver/solvers) []
- Clean up example/make it not insane(its so narsty at the moment) []
- Fix old resolver(allow for non clipping checks/contact generation on Poly/Poly collisions) []
- Clean up comments []