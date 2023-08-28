Name:      warwick-qhy-camera-data
Version:   20230828
Release:   0
Url:       https://github.com/warwick-one-metre/qhy-camd
Summary:   Camera configuration for the Windmill Hill Observatory telescope.
License:   GPL-3.0
Group:     Unspecified
BuildArch: noarch

%description

%build
mkdir -p %{buildroot}%{_sysconfdir}/camd
%{__install} %{_sourcedir}/config/warwick.json %{buildroot}%{_sysconfdir}/camd

%files
%defattr(0644,root,root,-)
%{_sysconfdir}/camd/warwick.json

%changelog
