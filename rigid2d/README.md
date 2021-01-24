# Rigid 2D transformation Library
A library for handling transformations in SE(2).

# Conceptual Questions
1. What is the difference between a class and a struct in C++?
    - A class has members as private by default while a struct has members as public by default.
    - A class will inherit members as private by default while a struct will inherit members as public by default

2. Why is Vector2D a struct and Transform2DClass (refer to at least 2 specic C++ core guidelines in your answer)?
    - According to C.2 in the guidelines, members in struct Vector2D can vary independently while members in class Transform2D are used together for update and calculation, which is invariant.
    - According to C.8 in the guidelines, Vector 2D does not need any non-public member, so it is a struct. Transform2D has non-public members to prevent writing from the outside, so it is a class.

3. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)?
    - According to C.46 in the guidelines, we would like to avoid uninteded implicit conversions in a single argument constructor.

4. We need to be able to ~normalize~ Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):
   - Propose three different designs for implementing the ~normalize~ functionality
   - Discuss the pros and cons of each proposed method, in light of the C++ Core Guidelines.
   - Which of the methods would you implement and why?
5. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not (Refer to C++ core guidelines in your answer)?
   - Refer to [[https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#con-constants-and-immutability][C++ Core Guidelines (Constants and Immutability)]] in your answer