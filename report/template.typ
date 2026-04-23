


#let appendix(
  title: "Appendices",
  title-size: 32pt,
  numbering-style: "A",
  body,
) = [
  #pagebreak()
  #align(center + horizon)[
    #text(size: title-size, weight: "bold")[#title]
  ]
  #pagebreak()

  #counter(heading).update(0)
  #set heading(numbering: (..nums) => "Appendix " + numbering(numbering-style, ..nums) + ":")
  // #set heading(numbering: "A.", supplement: [])

  #show heading.where(level: 1): it => {
    pagebreak(weak: true)
    it
  }

  #body
]
