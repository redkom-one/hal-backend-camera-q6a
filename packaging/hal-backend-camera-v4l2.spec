%define enable_libv4l2 1

Name:       hal-backend-camera-v4l2
Summary:    Tizen Camera Hal using generic V4L2 interface
Version:    0.0.1
Release:    0
Group:      Multimedia/Libraries
License:    Apache-2.0
Source0:    %{name}-%{version}.tar.gz
Requires(post): /sbin/ldconfig
Requires(postun): /sbin/ldconfig
BuildRequires:  pkgconfig(glib-2.0)
BuildRequires:  pkgconfig(libtbm)
BuildRequires:  pkgconfig(dlog)
BuildRequires:  pkgconfig(hal-api-common)
BuildRequires:  pkgconfig(hal-api-camera)
%if 0%{?enable_libv4l2}
BuildRequires:  pkgconfig(libv4l2)
%endif

%description
Tizen Camera Hal using generic V4L2 interface.


%prep
%setup -q


%build
./autogen.sh
%configure \
%if 0%{?enable_libv4l2}
	--enable-libv4l2\
%endif
	--disable-static\
	--libdir=%{_hal_libdir}
make %{?jobs:-j%jobs}

%install
%make_install
mkdir -p %{buildroot}%{_hal_licensedir}/%{name}
cp LICENSE.APLv2 %{buildroot}%{_hal_licensedir}/%{name}


%post
/sbin/ldconfig

%postun -p /sbin/ldconfig

%files
%manifest %{name}.manifest
%{_hal_licensedir}/%{name}/*
%defattr(-,root,root,-)
%{_hal_libdir}/*.so

