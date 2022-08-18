Name:      clasp-qhy-camera-data
Version:   20220818
Release:   0
Url:       https://github.com/warwick-one-metre/qhy-camd
Summary:   Camera configuration for the CLASP telescope.
License:   GPL-3.0
Group:     Unspecified
BuildArch: noarch

%description

%build
mkdir -p %{buildroot}%{_sysconfdir}/camd
%{__install} %{_sourcedir}/clasp/cam1.json %{buildroot}%{_sysconfdir}/camd
%{__install} %{_sourcedir}/clasp/cam2.json %{buildroot}%{_sysconfdir}/camd

%files
%defattr(0644,root,root,-)
%{_sysconfdir}/camd/cam1.json
%{_sysconfdir}/camd/cam2.json

%changelog
