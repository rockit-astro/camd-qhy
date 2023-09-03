Name:      rockit-camera-qhy
Version:   %{_version}
Release:   1
Summary:   Control code for a QHY600 CMOS camera
Url:       https://github.com/rockit-astro/qhy_camd
License:   GPL-3.0
BuildArch: noarch

%description


%build
mkdir -p %{buildroot}%{_bindir}
mkdir -p %{buildroot}%{_unitdir}
mkdir -p %{buildroot}%{_sysconfdir}/camd
mkdir -p %{buildroot}%{_udevrulesdir}

%{__install} %{_sourcedir}/qhy_camd %{buildroot}%{_bindir}
%{__install} %{_sourcedir}/qhy_camd@.service %{buildroot}%{_unitdir}

%{__install} %{_sourcedir}/config/halfmetre.json %{buildroot}%{_sysconfdir}/camd
%{__install} %{_sourcedir}/config/warwick.json %{buildroot}%{_sysconfdir}/camd
%{__install} %{_sourcedir}/config/clasp/cam1.json %{buildroot}%{_sysconfdir}/camd/clasp_cam1.json
%{__install} %{_sourcedir}/config/superwasp/cam1.json %{buildroot}%{_sysconfdir}/camd/superwasp_cam1.json
%{__install} %{_sourcedir}/config/superwasp/cam2.json %{buildroot}%{_sysconfdir}/camd/superwasp_cam2.json
%{__install} %{_sourcedir}/config/superwasp/cam3.json %{buildroot}%{_sysconfdir}/camd/superwasp_cam3.json
%{__install} %{_sourcedir}/config/superwasp/cam4.json %{buildroot}%{_sysconfdir}/camd/superwasp_cam4.json

%package server
Summary:  QHY camera server
Group:    Unspecified
Requires: python3-rockit-camera-qhy libqhyccd
%description server

%files server
%defattr(0755,root,root,-)
%{_bindir}/qhy_camd
%defattr(0644,root,root,-)
%{_unitdir}/qhy_camd@.service

%package data-clasp
Summary: QHY camera data for the CLASP telescope
Group:   Unspecified
%description data-clasp

%post data-clasp
mv %{_sysconfdir}/camd/clasp_cam1.json %{_sysconfdir}/camd/cam1.json

%preun data-clasp
mv %{_sysconfdir}/camd/cam1.json %{_sysconfdir}/camd/clasp_cam1.json

%files data-clasp
%defattr(0644,root,root,-)
%{_sysconfdir}/camd/clasp_cam1.json

%package data-halfmetre
Summary: QHY camera data for the half metre telescope
Group:   Unspecified
%description data-halfmetre

%files data-halfmetre
%defattr(0644,root,root,-)
%{_sysconfdir}/camd/halfmetre.json

%package data-superwasp
Summary: QHY camera data for the SuperWASP telescope
Group:   Unspecified
%description data-superwasp

%post data-superwasp
mv %{_sysconfdir}/camd/superwasp_cam1.json %{_sysconfdir}/camd/cam1.json
mv %{_sysconfdir}/camd/superwasp_cam2.json %{_sysconfdir}/camd/cam2.json
mv %{_sysconfdir}/camd/superwasp_cam3.json %{_sysconfdir}/camd/cam3.json
mv %{_sysconfdir}/camd/superwasp_cam4.json %{_sysconfdir}/camd/cam4.json

%preun data-superwasp
mv %{_sysconfdir}/camd/cam1.json %{_sysconfdir}/camd/superwasp_cam1.json
mv %{_sysconfdir}/camd/cam2.json %{_sysconfdir}/camd/superwasp_cam2.json
mv %{_sysconfdir}/camd/cam3.json %{_sysconfdir}/camd/superwasp_cam3.json
mv %{_sysconfdir}/camd/cam4.json %{_sysconfdir}/camd/superwasp_cam4.json

%files data-superwasp
%defattr(0644,root,root,-)
%{_sysconfdir}/camd/superwasp_cam1.json
%{_sysconfdir}/camd/superwasp_cam2.json
%{_sysconfdir}/camd/superwasp_cam3.json
%{_sysconfdir}/camd/superwasp_cam4.json

%package data-warwick
Summary: QHY camera data for Windmill Hill observatory
Group:   Unspecified
%description data-warwick

%files data-warwick
%defattr(0644,root,root,-)
%{_sysconfdir}/camd/warwick.json

%changelog
