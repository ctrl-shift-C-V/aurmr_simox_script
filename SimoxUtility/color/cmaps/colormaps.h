#pragma once

#include <SimoxUtility/color/ColorMap.h>


namespace simox::color::cmaps
{

    // Source: https://matplotlib.org/3.1.0/tutorials/colors/colormaps.html


    // Perceptually Uniform Sequential

    ColorMap viridis();
    ColorMap plasma();
    ColorMap inferno();
    ColorMap magma();
    ColorMap cividis();


    // Sequential

    ColorMap Greys();
    ColorMap Purples();
    ColorMap Blues();
    ColorMap Greens();
    ColorMap Oranges();
    ColorMap Reds();
    ColorMap YlOrBr();
    ColorMap YlOrRd();
    ColorMap OrRd();
    ColorMap PuRd();
    ColorMap RdPu();
    ColorMap BuPu();
    ColorMap GnBu();
    ColorMap PuBu();
    ColorMap YlGnBu();
    ColorMap PuBuGn();
    ColorMap BuGn();
    ColorMap YlGn();


    // Sequential (2)

    ColorMap binary();
    ColorMap gray();
    ColorMap bone();
    ColorMap pink();
    ColorMap spring();
    ColorMap summer();
    ColorMap autumn();
    ColorMap winter();
    ColorMap cool();
    ColorMap Wistia();
    ColorMap hot();
    ColorMap copper();


    // Diverging

    ColorMap PiYG();
    ColorMap PRGn();
    ColorMap BrBG();
    ColorMap PuOr();
    ColorMap RdGy();
    ColorMap RdBu();
    ColorMap RdYlBu();
    ColorMap RdYlGn();
    ColorMap Spectral();
    ColorMap coolwarm();
    ColorMap bwr();
    ColorMap seismic();


    // Cyclic

    ColorMap twilight();
    ColorMap twilight_shifted();
    ColorMap hsv();


    // Qualitative

    ColorMap Pastel1();
    ColorMap Pastel2();
    ColorMap Paired();
    ColorMap Accent();
    ColorMap Dark2();
    ColorMap Set1();
    ColorMap Set2();
    ColorMap Set3();
    ColorMap tab10();
    ColorMap tab20();
    ColorMap tab20b();
    ColorMap tab20c();

}

