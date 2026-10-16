-- include subprojects
includes("lib/commonlibsse-ng")

-- set project constants
set_project("CatCarry")
set_version("2.1.0")
set_license("GPL-3.0")
set_languages("c++23")
set_warnings("allextra")

-- add common rules
add_rules("mode.debug", "mode.releasedbg")

add_requires("nlohmann_json")

-- define targets
target("CatCarry")
    set_version("2.1.0")
    add_rules("commonlibsse-ng.plugin", {
        name = "CatCarry",
        author = "W1ter",
        description = "Crimson Desert style: pick up House Cats and carry them in your arms"
    })

    add_packages("nlohmann_json")

    add_files("src/**.cpp")
    add_headerfiles("src/**.h")
    add_includedirs("src")
    set_pcxxheader("src/pch.h")

    -- third party: SKSE Menu Framework API (header only)
    add_headerfiles("lib/SKSEMenuFramework/*.h")
    add_includedirs("lib/SKSEMenuFramework")
