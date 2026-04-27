// #import "@preview/retrofit:0.1.2": backrefs


// #show: backrefs.with(
//   format: links => text(gray)[(Cited on p. #links.join(", ", last: " and "))],
//   read: path => read(path),
// )

// Call once before the first #appendix() to emit the divider page and
// reset heading numbering for the appendix section.
#let begin-appendices(
  title: "Appendices",
  numbering-style: "A",
) = [
  #page(
    margin: (x: 2cm, y: 3cm),
    header: none,
    footer: none,
  )[
    #align(center + horizon)[
      #line(length: 60%, stroke: 0.5pt)
      #v(1.2em)
      #text(size: 26pt, weight: "bold")[#title]
      #v(1.2em)
      #line(length: 60%, stroke: 0.5pt)
    ]
  ]
  #counter(heading).update(0)
]

// Wrap each individual appendix section.
#let appendix(
  numbering-style: "A",
  body,
) = [
  #pagebreak(weak: true)
  #set heading(
    numbering: (..nums) => {
      let n = nums.pos()
      if n.len() == 1 { "Appendix " + numbering(numbering-style, ..nums) }
      else { numbering(numbering-style + ".1", ..nums) }
    },
    supplement: [],
  )
  #show heading.where(level: 1): it => {
    pagebreak(weak: true)
    counter(heading).display(it.numbering)
    [: ]
    it.body
  }
  #body
]

#let title-page(
  title: "",
  subtitle: "",
  course: "",
  author: "",
  date: "",
) = {
  page(
    margin: (x: 2cm, y: 3cm),
    header: none,
    footer: none,
  )[
    #align(center)[
      #v(1fr)

      #text(size: 13pt, weight: "regular", tracking: 1.5pt)[
        #upper(course)
      ]

      #v(1.2em)
      #line(length: 60%, stroke: 0.5pt)
      #v(1.2em)

      #text(size: 26pt, weight: "bold")[#title]

      #if subtitle != "" [
        #v(0.6em)
        #text(size: 14pt, style: "italic")[#subtitle]
      ]

      #v(1.2em)
      #line(length: 60%, stroke: 0.5pt)

      #v(2fr)

      #text(size: 12pt)[#author]
      #v(0.4em)
      #text(size: 11pt, fill: luma(80))[#date]

      #v(1fr)
    ]
  ]
  // restart page counter so content begins at page 1
  counter(page).update(1)
}

#let template(body) = [
  #set text(font: "New Computer Modern", size: 11pt)
  #set math.equation(numbering: "(1)")
  #set par(justify: true, leading: 0.65em)
  #set heading(numbering: "1.1")
  #show link: underline

  
  #set page(
    header: context {
      if counter(page).get().first() > 0 [
        #set text(size: 9pt, fill: luma(130))
        #smallcaps[RBE 550 — Motion Planning]
        #h(1fr)
        Liam Jennings
        #v(-0.6em)
        #line(length: 100%, stroke: 0.4pt + luma(180))
      ]
    },
    footer: context {
      if counter(page).get().first() > 0 [
        #line(length: 100%, stroke: 0.4pt + luma(180))
        #v(-0.6em)
        #set text(size: 9pt, fill: luma(130))
        #h(1fr)
        #counter(page).display("1")
        #h(1fr)
      ]
    },
  )

  #body
]


#let code-block(body) = rect(
  fill: luma(245), // light grey background
  stroke: 0.5pt + luma(200), // subtle border
  radius: 4pt, // rounded corners
  inset: 8pt, // padding inside the box
  body,
)