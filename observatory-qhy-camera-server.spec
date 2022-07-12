Name:      observatory-qhy-camera-server
Version:   20220712
Release:   0
Url:       https://github.com/warwick-one-metre/qhy-camd
Summary:   Control server for QHY600 CMOS camera
License:   GPL-3.0
Group:     Unspecified
BuildArch: noarch
Requires:  python3, python3-Pyro4, python3-numpy, python3-astropy, python3-warwick-observatory-common, python3-warwick-observatory-qhy-camera
Requires:  observatory-log-client, %{?systemd_requires}

%description

%build
mkdir -p %{buildroot}%{_bindir}
mkdir -p %{buildroot}%{_unitdir}
mkdir -p %{buildroot}%{_udevrulesdir}

%{__install} %{_sourcedir}/qhy_camd %{buildroot}%{_bindir}
%{__install} %{_sourcedir}/qhy_camd@.service %{buildroot}%{_unitdir}

%files
%defattr(0755,root,root,-)
%{_bindir}/qhy_camd
%defattr(0644,root,root,-)

%{_unitdir}/qhy_camd@.service

%changelog
