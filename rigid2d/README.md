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
   
    1. Create a single stand-alone function in the same namespace but does not belong to any class/struct
    2. Create a unit vector class. Such class will be constructed based on a Vector2D object. The object variable will be normalized and store in the private variable.
    3. Inherit a unit vector struct from the Vector2D object. The struct will have a constructor that turns a Vector2D object into a unit vector. 
    
   - Discuss the pros and cons of each proposed method, in light of the C++ Core Guidelines.
   
    1.
        - pros: According to C.5, helper function should be placed in the same namespace as the class/struct they support. The <b>normalize</b> function is a helper for the struct Vector2D, so it is the simplest solution to just put the function within the same namespace as Vector2D.
        - cons: Another person who uses the Vector2D object might not realize that there is a helper function for it since it is not within the object struct.
        
    2.
        - pros: According to C.2, we should use class if the class has an invariant. The x and y component of a unit vector are not independent, so a class is a good way to impose the private invariant constraint on the normal vector.
        - cons: Using a class could be an overkilled for such a simple function.
    
    3.
        - pros: By inheriting from the Vector2D struct, the new struct can keep the similar internal structure as the Vector2D struct. In the case where there could be new method added to the Vector2D struct, the new unit vector struct can also use it.
        - cons: According to C.2 in the guidelines, this is a bad practice. If another person accidentally change either x or y value of the unit vector struct object, the unit vector condition will not hold. 
        
        
   - Which of the methods would you implement and why?
   
     I will choose method 1. Normalizing a Vector2D object is a simple task just like deg2rad or rad2deg. It is not necessary to create a new structure for such a function. Therefore, it is better to keep it simple. Since it is a helper function for Vector2D, it will be in the same namespace as the Vector2D struct.
   
   
5. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not (Refer to C++ core guidelines in your answer)?

    - According to Con.2, "A member function should be marked const unless it changes the object’s observable state". In this case, Transform2D::inv() does not change any state of the objects within the class. It is only using the current object state to create a new Transform object that is inversed from the current one. For Transform2D::operator*=(), after the calculation, the private variables within the class will be updated or modifies by the calculation result, so the function should not declared const.
   
