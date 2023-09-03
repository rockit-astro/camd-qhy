RPMBUILD = rpmbuild --define "_topdir %(pwd)/build" \
        --define "_builddir %{_topdir}" \
        --define "_rpmdir %{_topdir}" \
        --define "_srcrpmdir %{_topdir}" \
        --define "_sourcedir %(pwd)"

all:
	mkdir -p build
	date --utc +%Y%m%d%H%M%S > VERSION
	${RPMBUILD} --define "_version %(cat VERSION)" -ba rockit-camera-qhy.spec
	${RPMBUILD} --define "_version %(cat VERSION)" -ba python3-rockit-camera-qhy.spec

	mv build/noarch/*.rpm .
	rm -rf build VERSION

install:
	@date --utc +%Y%m%d%H%M%S > VERSION
	@python3 -m build --outdir .
	@sudo pip3 install rockit.camera.qhy-$$(cat VERSION)-py3-none-any.whl
	@rm VERSION
	@sudo cp qhy_camd /bin/
	@sudo cp qhy_camd@.service /usr/lib/systemd/system/
	@sudo install -d /etc/qhy_camd
	@echo ""
	@echo "Installed server and service files."
	@echo "Now copy the relevant json config files to /etc/qhy_camd/"
