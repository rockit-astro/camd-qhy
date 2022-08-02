Name:      superwasp-qhy-camera-data
Version:   20220802
Release:   0
Url:       https://github.com/warwick-one-metre/qhy-camd
Summary:   Camera configuration for the SuperWASP telescope.
License:   GPL-3.0
Group:     Unspecified
BuildArch: noarch

%description

%build
mkdir -p %{buildroot}%{_sysconfdir}/camd
%{__install} %{_sourcedir}/superwasp/cam1.json %{buildroot}%{_sysconfdir}/camd
%{__install} %{_sourcedir}/superwasp/cam2.json %{buildroot}%{_sysconfdir}/camd
%{__install} %{_sourcedir}/superwasp/cam3.json %{buildroot}%{_sysconfdir}/camd
%{__install} %{_sourcedir}/superwasp/cam4.json %{buildroot}%{_sysconfdir}/camd

%files
%defattr(0644,root,root,-)
%{_sysconfdir}/camd/cam1.json
%{_sysconfdir}/camd/cam2.json
%{_sysconfdir}/camd/cam3.json
%{_sysconfdir}/camd/cam4.json

%changelog
