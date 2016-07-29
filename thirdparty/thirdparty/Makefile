all:
	@echo "making third party libs"
	cd libcvd && make
	cd gvars3 && make
	cd TooN && make
	cd eigen-3.2.1/build && make
	echo "134"
	cd openFabMap && make


clean:
	@echo "cleaning up third party libs"
	cd libcvd && make clean
	cd gvars3 && make clean

eclipse-project:
	@echo "skip thirdparty."

