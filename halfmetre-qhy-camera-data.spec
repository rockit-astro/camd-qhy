Name:      halfmetre-qhy-camera-data
Version:   20230624
Release:   0
Url:       https://github.com/warwick-one-metre/qhy-camd
Summary:   Camera configuration for the Half metre telescope.
License:   GPL-3.0
Group:     Unspecified
BuildArch: noarch

%description

%build
mkdir -p %{buildroot}%{_sysconfdir}/camd
%{__install} %{_sourcedir}/halfmetre/halfmetre.json %{buildroot}%{_sysconfdir}/camd

%files
%defattr(0644,root,root,-)
%{_sysconfdir}/camd/halfmetre.json

%changelog
