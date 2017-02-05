1. Create a folder for the project. Let's call it ````projectPath````. For example, ````projectPath```` = ````/home/user/projectPath````.

2. In ````projectPath````, create a file with extension ````cc```` (others probably work, but weren't tested). Let's call it ````sourceFile````. For example, ````sourceFile```` = ````main.cc````.

3. In ````projectPath````, create a file called ````CMakeLists.txt````, and paste in it everything after this line until the line that stars with ````4.````:
    ````
    cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
    
    find_package(gazebo REQUIRED)
    include_directories(${GAZEBO_INCLUDE_DIRS})
    link_directories(${GAZEBO_LIBRARY_DIRS})
    list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
    
    add_library(main SHARED main.cc)
    target_link_libraries(main ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
    ````
4. In this file, in the lines that start with ````add_library```` and ````target_link_libraries````, notice that ````main```` is related to the ````sourceFile````. Adjust these lines accordingly.

5. In ````projectPath````, create a folder called ````build````.

6. Open a terminal inside of ````build```` and execute the following line:

    ````cmake ../````

7. Create a new project in NetBeans. Under **C/C++**, choose **C/C++ Project With Existing Sources**.

8. Click **Browse...** and choose ````projectPath````. Choose **Custom** and click **Next >**.

9. Choose **Using an existing makefile**, click **Browse...** and select ````projectPath/build/Makefile````. **Clean and Build after Finish** is up to you. Click **Next >**.

10. In **Build Result**, click **Browse...** and choose where the output file will be generated and what will be it's name. Without NetBeans and with these examples, the default would be ````projectPath/build/libmain.so````. Click **Next >** 3 times.

11. If you change the project name or path, NetBeans' files will be generated separated from ````projectPath````, so leave them as they are. Click **Finish**.

12. Now you can rename the project without causing the folder structure to change, but remember to leave **Also Rename Project Folder** unchecked.

13. Now just (clean and) build the project and the output file will be generated automatically.

14. Should you wish to delete the NetBeans' project, beware that the source files will also be deleted if you check **Also delete sources under ... folder**.
