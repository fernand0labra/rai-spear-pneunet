FROM fernand0labra/sofa:base.v24.12

ENV DEBIAN_FRONTEND=noninteractive

# Set environment variable
ENV QT_QPA_PLATFORM=xcb

# Install required compiler and build SOFA with selected plugins
RUN apt-get update && \
    apt-get install -y gcc-10 g++-10 ninja-build && \
    cd /sofa && \
    cmake -S /sofa/src -B /sofa/build -G Ninja \
          -DCMAKE_C_COMPILER=/usr/bin/gcc-10 \
          -DCMAKE_CXX_COMPILER=/usr/bin/g++-10 \
          -DCMAKE_BUILD_TYPE=RelWithDebInfo \
          -DSOFA_EXTERNAL_DIRECTORIES=/sofa/plugins \
          -DSOFA_FETCH_CGALPLUGIN=ON \
          -DPLUGIN_CGALPLUGIN=ON \
          -DPLUGIN_BEAMADAPTER=ON \
          -DPLUGIN_COSSERAT=ON \
          -DPLUGIN_SOFACUDA=ON \
          -DPLUGIN_MODELORDERREDUCTION=ON \
          -DPLUGIN_SOFAPYTHON3=ON \
          -DPLUGIN_SOFTROBOTS=ON \
          -DPLUGIN_SPLIB=ON \
          -DPLUGIN_STLIB=ON && \
    cmake --build /sofa/build && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /sofa
