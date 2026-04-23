#import "@preview/retrofit:0.1.2": backrefs

// #show: backrefs.with(
//   format: links => text(gray)[(Cited on p. #links.join(", ", last: " and "))],
//   read: path => read(path),
// )

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

#let template(body) = [
  #set text(font: "New Computer Modern", size: 11pt)
  #set math.equation(numbering: "(1)")
  #set par(justify: true, leading: 0.65em)
  #show link: underline
  #body
]


#let code-block(body) = rect(
  fill: luma(245), // light grey background
  stroke: 0.5pt + luma(200), // subtle border
  radius: 4pt, // rounded corners
  inset: 8pt, // padding inside the box
  body,
)