
ACLOCAL_AMFLAGS = -I ./m4

SUBDIRS = src

jenkins: bear clangtidy cppcheck  docs	

# No unit tests at this moment
#unittests:		
#	./TestExe1/testexe1 --log_format=XML --log_sink=boost-result.xml --log_level=all --report_level=no --result_code=no

# No unit tests at this moment
#testcoverage:	
#	- ./TestExe1/testexe1 # "-" results to ignore the result of the command
#	lcov --directory ./TestExe1 --gcov-tool /usr/bin/gcov --capture --output-file gcov.info
#	genhtml ./gcov.info --output-directory ./lcov

if HAVE_DOXYGEN
docs: doxygen

doxygen:
		mkdir -p $(top_srcdir)/docs/generated
		cd .. ; $(DOXYGEN) $(abs_top_builddir)/Doxyfile
endif

if HAVE_CLANGTIDY
clangtidy:
		$(CLANGTIDY) -quiet -j9 > clang-tidy-result.txt
		echo "<html><br></body> `sed -z 's/\n/<br>/g' clang-tidy-result.txt`</html><br>/<body>" > clang-tidy.html
endif

if HAVE_BEAR
bear:
		make clean
		$(BEAR) -- make
endif

if HAVE_COMPILEDB
compiledb:
		make clean
		$(COMPILEDB) -n make --dry-run
endif

if HAVE_CPPCHECK
cppcheck:	
		cppcheck	--xml --xml-version=2	\
							--enable=all	\
							--inconclusive	\
							--force	\
				 			--inline-suppr	\
				 			--std=c++20	\
				 			--suppress=missingIncludeSystem	\
				 			--library=boost,wxwidgets	\
				 			-I ../src	\
				 			../src 2> cppcheck-result.xml
		echo "<html><br></body><br><xmp>`cat cppcheck-result.xml` </xmp></html><br></body>"  > cppcheck-result.html				 			

cppcheck_checkconfig:														
		cppcheck	--enable=all	\
							--inconclusive	\
							--force	\
				 			--inline-suppr	\
				 			--std=c++20	\
				 			--suppress=missingIncludeSystem	\
				 			--library=boost,wxwidgets	\
				 			-I ../src	\
				 			--check-config	\
				 			../src
endif
	
distclean-local:
	-rm -rf $(top_srcdir)/build/src
	-rm -rf $(top_srcdir)/linux/src
	-rm -rf $(top_srcdir)/mingw/src
	-rm -rf $(top_srcdir)/darwin/src
	-rm -rf $(top_srcdir)/autom4te.cache
	-rm -rf $(top_srcdir)/docs/generated