# Submission checklist — IOP Engineering Research Express (ERX)

Target: IOP Publishing, *Engineering Research Express*. Broad-scope engineering
journal; rigour and reproducibility are weighted above novelty and impact.

## Venue requirements

- IMRaD structure (Introduction, Method, Results, Discussion, Conclusion). Done.
- Abstract of at most 300 words. Done (293).
- Keywords supplied. Done (7).
- No mandatory template; a single PDF with figures embedded in the text.
- Figure captions self-contained, not relying on colour alone.
- References in Vancouver numerical or Harvard style, applied consistently. Done (Vancouver).
- Supplementary material and code/data deposition encouraged.

## Compliance verified against the IOP/ERX guidelines (2026-10-15)

- [x] Single PDF for submission, figures and tables embedded in the text
- [x] Authors' full names and institutes at the start of the PDF (single-anonymous model)
- [x] Body text at 12 pt with standard line spacing
- [x] Abstract 293 words, no references or undefined acronyms
- [x] Vancouver numerical references, in square brackets, listed in order
- [x] Tables without colour, each with a concise caption
- [x] Acknowledgements before the References, with funding, contributions and competing interests
- [x] Ethics statement (simulation only, no human or animal subjects)
- [x] No CJK characters in the body text
- [x] Cover letter (`manuscript/cover_letter.md`)
- [x] Repository URL in the Data and Code Availability statement

## Manuscript package

- [x] Title, abstract, keywords
- [x] Sections 1 to 8 (`RA-DWA_manuscript.md`, and `paper.html` which produces the PDF)
- [x] Tables 1 to 3 (`tables_1_2_3.md`) and Tables 4 to 8 (`results/table*.md`, `results/stress.csv`, `results/sumo_validation.md`)
- [x] Figures 1 to 13 (`figures/`)
- [x] Figure and table captions
- [x] References verified, none fabricated (`references_verified.md`)
- [x] Reproducibility checks (`scripts/verify.py`, 33 of 33)

## Details to confirm at submission

- [x] **Authors**: Suihua Lin and Yuelan Huang (corresponding), School of Mathematics and Computer Science, Guangxi Minzu Normal University, Chongzuo 532200, Guangxi, China. Email: zhuyi19840719@163.com.
- [ ] **ORCIDs**: not yet supplied; recommended by IOP for all authors.
- [x] **Funding**: stated as "This research received no external funding". Confirm, or replace with the grant names and numbers.
- [x] **Author contributions**: stated in CRediT form. Confirm as accurate.
- [x] **Research interests** for the submission form: S. Lin, applied mathematics, mathematical modelling and optimisation, with applications to motion planning; Y. Huang, computational mathematics, algorithm design and optimisation, with applications to motion planning.
- [x] **Repository**: https://github.com/kksuobo/ERX
- [ ] **Suggested reviewers** (optional) and any opposed reviewers.
- [x] **Competing interests**: stated as none.

## Remaining production steps

- [ ] Final language pass and a typo and number cross-check against `results/`.
- [ ] Confirm the peer-review model on the submission site (single-anonymous) before upload.
- [ ] Decide the open-access option and confirm any article publishing charge at submission.
