RobotWorld building and using (readme)
--------------------------------------

This readme contains several sections. Please read them. It makes your life easier. Really. 

RobotWorld is an example!
-------------------------

RobotWorld is not just part of an exam. It is also an example of how to do things. Please study it as such.
Many of the examples are *simple* examples and not really industrial grade solutions for real world projects.
But the real world solutions are most of the time *extensions* of the examples given here, not radically different
solutions. So you can learn from them. 

Source
------

The source code is documented using Doxygen (see <https://www.doxygen.nl/>). To generate the documentation
run "make docs" (*after* running configure, see below). This will generate the documentation. After the generation the main 
entry file is [docs/generated/html/index.html](./docs/generated/html/index.html). For more (technical) documentation
please generate the documentation and read that.

Compilation
-----------

-   This readme assumes that both Boost and wxWidgets are compiled and
    installed using the OSM installation manual (see <http://onderwijsonline.han.nl>).

-   "`\`" at the and of a line, means that the actual command line  continues with the next line!

-   All command sequences start from the root of the project, `$ROBOTWORLD_ROOT` in the examples

-   Adjust all paths to taste, but *DO NOT* use spaces in any path

-   Adjust for any Boost and wxWidgets versions

-   This readme uses wxWidgets-x.x.x and for Boost-x.xx to explain the procedure. Replace the x's according to the version you use. Please use the versions as per the "OSM Installatiehandleiding". 

-   On Windows, use a MinGW MSys command shell, *NOT* a Windows cmd shell.

-   If the configure script is missing or out of sync with the installed MSys or Linux Autotools files, run the "`autogen.sh`" script in the root of RobotWorld (use "`./autogen.sh`").
    

### Compilation on Linux

If both Boost and wxWidgets have been compiled and installed as described in the installation manual, the following commands can be used to configure and build RobotWorld.

Navigate to the root folder of RobotWorld (for example "`~/sources/robotworld/`", where "`~`" stands for the home folder of the
user.

    cd ~/sources/robotworld/

    mkdir linux

    cd linux

    ../configure --with-cxx=17

    make -j4 2>&1 | tee build.log

Notes:

-   With the command line option `--with-cxx=` it is possible to change the target C++-standard. For more information run
    "`../configure --help`".

-   If Boost isn't installed in the default location (not build with the `install` option) use the `--with-boost=<path_to_boost>` command line option. The `<path_to_boost>` must point to the root directory  of the installed/compiled Boost. Run "`../configure --help`" for  details on command line options.

-   If wxWidgets isn't installed in the default location (not build with the `install` option) use the
    `--with-wx-config=<path_to_wx_config_file>` command line option. The "`<path_to_wx_config_file>`" is the full path of the file `wx-config` file *including the file name*. If the installation manual was used,  the location should be "`~/sources/wxWidgets-x.x.x/build-linux/wx-config`". Run "`../configure --help`" for details on command line options.

### Compilation on Windows

If both Boost and wxWidgets have been compiled and installed as described in the installation manual, the following commands can be used to configure and build RobotWorld.

Open the MinGW MSys2 terminal (*not* the Windows command line). Navigate to the root folder of RobotWorld (for example "`/c/robotworld/`")

    cd /c/robotworld/

    mkdir mingw 

    cd mingw/

    ../configure --with-cxx=17  \
        --with-boost=/c/boost/  \
        --with-wx-config=/c/wxWidgets-x.x.x/mingw/wx-config

    make -j4 2>&1 | tee build.log

Notes:

-   With the command line option `--with-cxx=` it is possible to change  the target C++-standard. For more information run
    "`../configure --help`".

-   If Boost isn't installed in the location as used in the installation  manual on OnderwijsOnline, change the `--with-boost=<path_to_boost>` command line option. The `<path_to_boost>` must point to the root directory of the installed/compiled Boost (containing the "`lib`" and "`include`" folders of Boost). Run "`../configure --help`" for details on command line options.

-   If wxWidgets isn't installed in the default location (not build with the `install` option) use the`--with-wx-config=<path_to_wx_config_file>` command line option. The "`<path_to_wx_config_file>`" is the full path of the file
    `wx-config` file. If the installation manual was used, the location should be "`/c/wxWidgets-x.x.x/mingw/wx-config`". Run
    "`../configure --help`" for details on command line options.

### Cleaning the build

To clean the build area one can use the following commands:

-	"make clean": this removes the object and executable files. It does not remove any Doxygen generated files or any othe
    generated files.
-   "make distclean": this removes *all* generated files. Use this command prior to sending your project to a teacher.

Eclipse instructions
--------------------

### Create Eclipse project

RobotWorld uses a make file for compilation, it is possible to use
Eclipse in combination with a make file. Follow the next steps to create
a project in Eclipse.

1.  Open Eclipse and click "`New C/C++ Project`". Select "`C++ Managed Build`" and click "`Next`"(do not use the CMake or
    other *make* projects).

2.  In the next step ("`Create C++ project of selected type`"\*) do the following:

    a.  Give the project a name
    b.  Deselect "`Use default location`" and set the location to the
        root of RobotWorld (Linux: "`~/sources/robotworld/`", Windows:
        "`c:\robotworld\`")
    c.  Set project type (left below) to "`Makefile project -> empty`"
    d.  click "`Finish`"

3.  After creating the project, change the build location to the compilation directory used earlier building RobotWorld. Select "`Properties`" of the project, go to "`C/C++ Build`" and change the "`Build directory`" (use the button "`Workspace`" ) to the
compilation folder (Linux: "`~/sources/robotworld/linux`", Windows: "`c:\robotworld\mingw`"). Click on
 "`Apply and Close`".

To test if the Eclipse project is configured correctly, click "`Clean...`" and build the project again. Try to run RobotWorld from
Eclipse by clicking "`Ctrl+F11`" (or use the menu "`Run`").

### Indexer of Eclipse

If using Makefiles to compile programs Eclipse (i.e. its indexer with
auto-complete feature, code browsing etc) does not use auto-discovery
features for `#includes` and `#defines`. This must be configured
manually.

This must be done in the "Paths and Symbols" part of the project
settings: "`Project->properties->C/C++ General->Paths and Symbols`" on
the tabs "`includes`" and "`symbols`"`

on the tab includes in my case these needed to be added, make sure you select "add to all configurations" and "add to all languages":

C:\wxWidgets-x.x.x\mingw\lib\wx\include\msw-unicode-x.x
C:\wxWidgets-x.x.x\include
C:\boost\include

then go to the tab symbols and add these symbols with their values, make sure you select "add to all configurations" and "add to all languages":

__WXMSW__           with no value
_FILE_OFFSET_BITS   with value 64
WXUSINGDLL          with no value

Remarks:

-   If Boost and wxWidgets are installed, one only has to configure the wxWidgets defines (-D in the output, see below)

-   If one of them is not installed one has to configure that include directories also

    -   Use "`wx-config --cppflags`" for the relevant wxWidgets values

    -   Use the root directory of a staged Boost version

    -   Use "`-std=c++17`" for the indexer and not "`-std=gnu++17`" if the C++ standard headers are not resolved

-   Use "`Window->Show view->Other->C/C++ index`" to find any unresolved
    header files or unresolved other C/C++ symbols.

### Debugging in Eclipse (windows)

-   Both Boost and wxWidgets lib directory must have been added o the PATH variable.

-   Change the `C/C++ Application` launch target to `robotworld.exe` in the (hidden) folder `.libs`.
    
### Using static code checkers

The configure script checks for the availability of cppcheck and clang-tidy. As clang-tidy needs
a compilation database the configure script also checks if "bear" or "compiledb" are installed.

If cppcheck is available just run "make cppcheck".

If clang-tidy is available one first has to create a compilation database ("compile_commands.json").
Depending on having bear of compiledb run "make bear" or "make compiledb". After that run "make clangtidy"

Bear is installable on Debian: "apt install bear"
More info: <https://github.com/rizsotto/Bear>

For compiledb: Installation and usage instructions: see <https://github.com/nickdiego/compiledb>.

If clang-tidy uses colours and the "UseColor : false" in the .clang-tidy is ignored (version < 15) the colours can be stripped from clang-tidy-result.txt by: 
`cat clang-tidy-result.txt | sed 's/\x1B\[[0-9;]\{1,\}[A-Za-z]//g'` 
    
### Using Sanitizers

- If you compile with "-fsanitize=address -fsanitize=leak -fsanitize=undefined" you must link with "-fsanitize=address -fsanitize=leak -fsanitize=undefined" for linking with the correct libraries.
- If your IDE does not support that you can link with the libarries "asan" en "usan" (in that order!)
- If the LeakSanitizer gives false positives you can add those (after careful inspection!!!) to the LeakSanitizerSuppression.txt
