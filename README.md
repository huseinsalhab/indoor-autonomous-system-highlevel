## Self Driving Car CE 123

### Coding Standards Doc

#### Golden Rule
Search on google before asking for help.

#### Preamble
1. Every file, the first line will be your name, and date
2. Every file, the second line will be a short description purpose of the code written
3. Reserve 3-4 lines (multiline comment) for any links, resources, or help that you get while writing your code. The purpose is to keep tracks of links you use in case we run into similar problems again (VERY IMPORTANT).
4. For step 3, do this for every file, or for blocks of code where the links are relevant.
5. Make comments short, but with good explanation. Nothing too long, nothing too short. Comments are for things that are not obvious, or to highlight and explain salient (relevant) lines of code.

#### Specifics
1. USE RELEVANT NAMES FOR VARIABLES. Unless it is a super obvious abbreviation, or letter (ex. int "i" in a for loop, you dont need to name it "iterator".)
2. USE PARENTHESES. Do not rely on your knowledge of operator precedence. Use parentheses in complex logical structures (Logical, Boolean, Arithmetic operators). 
		Example:
		This is BAD: (var1 && var2 || var3 ^ var4)
		This is GOOD: ( (var1 && var2) || (var 3 ^ var4) )
3.  Everytime that you edit a file, make sure that you are using SPACES for TABS. Make sure UNIX line endings, unless you know what u are doing, no editing code on windows (go through cloud9, a VM, or your raspberry PI, or linux environment to edit code).
		:set ff=unix

<p align="center">
  <img src="https://cdn.pbrd.co/images/H5SkFiL.png" width="350"/>

</p>
![alt text](https://cdn.pbrd.co/images/H5SkFiL.png)

4. ALL FUNCTIONS, a comment above (multi or single line, just be consistent), listing return type, argument type, and EXPLANATIONS IN ORDER for ALL ARGUMENTS IN ORDER.
5. USE K&R braces and indentation style
		
		https://en.wikipedia.org/wiki/Indentation_style#K&R
		C/C++ SINGLE LINE IF STATEMENT ALWAYS HAS BRACES NO MATTER WHAT

#### Version Control
0. Make a branch for any features that you need to add. (Git checkout -b kelvin.new_feature)
1. The proper naming for a branch: 
		
		first_name.feature_name
		kelvin.adding_slam_module
2. GIT PULL before you start writing code.
3. Write a sensible commit statement (nothing too long, nothing too short). Im not too strict on this
4. Dont Commit for every little thing you write. Dont commit for HUGE updates. Be somewhere in the middle

		
	


