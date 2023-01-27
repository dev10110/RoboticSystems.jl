using RoboticSystems
using Documenter

DocMeta.setdocmeta!(RoboticSystems, :DocTestSetup, :(using RoboticSystems); recursive=true)

makedocs(;
    modules=[RoboticSystems],
    authors="Devansh Ramgopal Agrawal <devansh@umich.edu> and contributors",
    repo="https://github.com/dev10110/RoboticSystems.jl/blob/{commit}{path}#{line}",
    sitename="RoboticSystems.jl",
    format=Documenter.HTML(;
        prettyurls=get(ENV, "CI", "false") == "true",
        canonical="https://dev10110.github.io/RoboticSystems.jl",
        edit_link="main",
        assets=String[],
    ),
    pages=[
        "Home" => "index.md",
    ],
)

deploydocs(;
    repo="github.com/dev10110/RoboticSystems.jl",
    devbranch="main",
)
