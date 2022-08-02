RPMBUILD = rpmbuild --define "_topdir %(pwd)/build" \
        --define "_builddir %{_topdir}" \
        --define "_rpmdir %{_topdir}" \
        --define "_srcrpmdir %{_topdir}" \
        --define "_sourcedir %(pwd)"

GIT_VERSION = $(shell git name-rev --name-only --tags --no-undefined HEAD 2>/dev/null || echo git-`git rev-parse --short HEAD`)
SERVER_VERSION=$(shell awk '/Version:/ { print $$2; }' observatory-qhy-camera-server.spec)

all:
	mkdir -p build
	cp qhy_camd qhy_camd.bak
	awk '{sub("SOFTWARE_VERSION = .*$$","SOFTWARE_VERSION = \"$(SERVER_VERSION) ($(GIT_VERSION))\""); print $0}' qhy_camd.bak > qhy_camd
	${RPMBUILD} -ba observatory-qhy-camera-server.spec
	${RPMBUILD} -ba observatory-qhy-camera-client.spec
	${RPMBUILD} -ba python3-warwick-observatory-qhy-camera.spec
	${RPMBUILD} -ba clasp-qhy-camera-data.spec
	${RPMBUILD} -ba superwasp-qhy-camera-data.spec
	mv build/noarch/*.rpm .
	rm -rf build
	mv qhy_camd.bak qhy_camd

