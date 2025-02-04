# This file is generated by sources_gen script and then checked in.
# A linter cross-checks that the generated files is up-to-date.
#
# To regenerate this file:
# $ cd drake
# $ bazel build //tools/workspace/lapack_internal:gen/sources.bzl
# $ cp bazel-bin/tools/workspace/lapack_internal/gen/sources.bzl \
#       tools/workspace/lapack_internal/lock/sources.bzl

DBLAS1 = [
    "dasum.f",
    "daxpy.f",
    "dcopy.f",
    "ddot.f",
    "dnrm2.f90",
    "drot.f",
    "drotg.f90",
    "drotm.f",
    "drotmg.f",
    "dscal.f",
    "dsdot.f",
    "dswap.f",
    "idamax.f",
]

ALLBLAS = [
    "lsame.f",
    "xerbla.f",
    "xerbla_array.f",
]

DBLAS2 = [
    "dgbmv.f",
    "dgemv.f",
    "dger.f",
    "dsbmv.f",
    "dspmv.f",
    "dspr.f",
    "dspr2.f",
    "dsymv.f",
    "dsyr.f",
    "dsyr2.f",
    "dtbmv.f",
    "dtbsv.f",
    "dtpmv.f",
    "dtpsv.f",
    "dtrmv.f",
    "dtrsv.f",
]

DBLAS3 = [
    "dgemm.f",
    "dgemmtr.f",
    "dsymm.f",
    "dsyr2k.f",
    "dsyrk.f",
    "dtrmm.f",
    "dtrsm.f",
]

ALLAUX = [
    "../INSTALL/ilaver.f",
    "../INSTALL/lsame.f",
    "../INSTALL/slamch.f",
    "chla_transtype.f",
    "ieeeck.f",
    "iladiag.f",
    "ilaenv.f",
    "ilaenv2stage.f",
    "ilaprec.f",
    "ilatrans.f",
    "ilauplo.f",
    "iparam2stage.F",
    "iparmq.f",
    "la_xisnan.F90",
    "lsamen.f",
    "xerbla.f",
    "xerbla_array.f",
]

DZLAUX = [
    "../INSTALL/dlamch.f",
    "../INSTALL/droundup_lwork.f",
    "dbdsdc.f",
    "dbdsqr.f",
    "ddisna.f",
    "disnan.f",
    "dlabad.f",
    "dlacpy.f",
    "dladiv.f",
    "dlae2.f",
    "dlaebz.f",
    "dlaed0.f",
    "dlaed1.f",
    "dlaed2.f",
    "dlaed3.f",
    "dlaed4.f",
    "dlaed5.f",
    "dlaed6.f",
    "dlaed7.f",
    "dlaed8.f",
    "dlaed9.f",
    "dlaeda.f",
    "dlaev2.f",
    "dlagtf.f",
    "dlagts.f",
    "dlaisnan.f",
    "dlamrg.f",
    "dlaneg.f",
    "dlanst.f",
    "dlapy2.f",
    "dlapy3.f",
    "dlarnv.f",
    "dlarra.f",
    "dlarrb.f",
    "dlarrc.f",
    "dlarrd.f",
    "dlarre.f",
    "dlarrf.f",
    "dlarrj.f",
    "dlarrk.f",
    "dlarrr.f",
    "dlartg.f90",
    "dlartgp.f",
    "dlartgs.f",
    "dlaruv.f",
    "dlas2.f",
    "dlascl.f",
    "dlasd0.f",
    "dlasd1.f",
    "dlasd2.f",
    "dlasd3.f",
    "dlasd4.f",
    "dlasd5.f",
    "dlasd6.f",
    "dlasd7.f",
    "dlasd8.f",
    "dlasda.f",
    "dlasdq.f",
    "dlasdt.f",
    "dlaset.f",
    "dlasq1.f",
    "dlasq2.f",
    "dlasq3.f",
    "dlasq4.f",
    "dlasq5.f",
    "dlasq6.f",
    "dlasr.f",
    "dlasrt.f",
    "dlassq.f90",
    "dlasv2.f",
    "dpttrf.f",
    "dstebz.f",
    "dstedc.f",
    "dsteqr.f",
    "dsterf.f",
    "la_constants.f90",
]

DSLASRC = [
    "sgetrf.f",
    "sgetrs.f",
    "spotrf.f",
    "spotrs.f",
]

DLASRC = [
    "dbbcsd.f",
    "dbdsvdx.f",
    "dgbbrd.f",
    "dgbcon.f",
    "dgbequ.f",
    "dgbequb.f",
    "dgbrfs.f",
    "dgbsv.f",
    "dgbsvx.f",
    "dgbtf2.f",
    "dgbtrf.f",
    "dgbtrs.f",
    "dgebak.f",
    "dgebal.f",
    "dgebd2.f",
    "dgebrd.f",
    "dgecon.f",
    "dgedmd.f90",
    "dgedmdq.f90",
    "dgeequ.f",
    "dgeequb.f",
    "dgees.f",
    "dgeesx.f",
    "dgeev.f",
    "dgeevx.f",
    "dgehd2.f",
    "dgehrd.f",
    "dgejsv.f",
    "dgelq.f",
    "dgelq2.f",
    "dgelqf.f",
    "dgelqt.f",
    "dgelqt3.f",
    "dgels.f",
    "dgelsd.f",
    "dgelss.f",
    "dgelst.f",
    "dgelsy.f",
    "dgemlq.f",
    "dgemlqt.f",
    "dgemqr.f",
    "dgemqrt.f",
    "dgeql2.f",
    "dgeqlf.f",
    "dgeqp3.f",
    "dgeqp3rk.f",
    "dgeqr.f",
    "dgeqr2.f",
    "dgeqr2p.f",
    "dgeqrf.f",
    "dgeqrfp.f",
    "dgeqrt.f",
    "dgeqrt2.f",
    "dgeqrt3.f",
    "dgerfs.f",
    "dgerq2.f",
    "dgerqf.f",
    "dgesc2.f",
    "dgesdd.f",
    "dgesv.f",
    "dgesvd.f",
    "dgesvdq.f",
    "dgesvdx.f",
    "dgesvj.f",
    "dgesvx.f",
    "dgetc2.f",
    "dgetf2.f",
    "dgetrf.f",
    "dgetrf2.f",
    "dgetri.f",
    "dgetrs.f",
    "dgetsls.f",
    "dgetsqrhrt.f",
    "dggbak.f",
    "dggbal.f",
    "dgges.f",
    "dgges3.f",
    "dggesx.f",
    "dggev.f",
    "dggev3.f",
    "dggevx.f",
    "dggglm.f",
    "dgghd3.f",
    "dgghrd.f",
    "dgglse.f",
    "dggqrf.f",
    "dggrqf.f",
    "dggsvd3.f",
    "dggsvp3.f",
    "dgsvj0.f",
    "dgsvj1.f",
    "dgtcon.f",
    "dgtrfs.f",
    "dgtsv.f",
    "dgtsvx.f",
    "dgttrf.f",
    "dgttrs.f",
    "dgtts2.f",
    "dhgeqz.f",
    "dhsein.f",
    "dhseqr.f",
    "dlabrd.f",
    "dlacn2.f",
    "dlacon.f",
    "dlaein.f",
    "dlaexc.f",
    "dlag2.f",
    "dlag2s.f",
    "dlags2.f",
    "dlagtm.f",
    "dlagv2.f",
    "dlahqr.f",
    "dlahr2.f",
    "dlaic1.f",
    "dlaln2.f",
    "dlals0.f",
    "dlalsa.f",
    "dlalsd.f",
    "dlamswlq.f",
    "dlamtsqr.f",
    "dlangb.f",
    "dlange.f",
    "dlangt.f",
    "dlanhs.f",
    "dlansb.f",
    "dlansf.f",
    "dlansp.f",
    "dlansy.f",
    "dlantb.f",
    "dlantp.f",
    "dlantr.f",
    "dlanv2.f",
    "dlaorhr_col_getrfnp.f",
    "dlaorhr_col_getrfnp2.f",
    "dlapll.f",
    "dlapmr.f",
    "dlapmt.f",
    "dlaqgb.f",
    "dlaqge.f",
    "dlaqp2.f",
    "dlaqp2rk.f",
    "dlaqp3rk.f",
    "dlaqps.f",
    "dlaqr0.f",
    "dlaqr1.f",
    "dlaqr2.f",
    "dlaqr3.f",
    "dlaqr4.f",
    "dlaqr5.f",
    "dlaqsb.f",
    "dlaqsp.f",
    "dlaqsy.f",
    "dlaqtr.f",
    "dlaqz0.f",
    "dlaqz1.f",
    "dlaqz2.f",
    "dlaqz3.f",
    "dlaqz4.f",
    "dlar1v.f",
    "dlar2v.f",
    "dlarf.f",
    "dlarf1f.f",
    "dlarf1l.f",
    "dlarfb.f",
    "dlarfb_gett.f",
    "dlarfg.f",
    "dlarfgp.f",
    "dlarft.f",
    "dlarfx.f",
    "dlarfy.f",
    "dlargv.f",
    "dlarmm.f",
    "dlarrv.f",
    "dlartv.f",
    "dlarz.f",
    "dlarzb.f",
    "dlarzt.f",
    "dlaswlq.f",
    "dlaswp.f",
    "dlasy2.f",
    "dlasyf.f",
    "dlasyf_aa.f",
    "dlasyf_rk.f",
    "dlasyf_rook.f",
    "dlat2s.f",
    "dlatbs.f",
    "dlatdf.f",
    "dlatps.f",
    "dlatrd.f",
    "dlatrs.f",
    "dlatrs3.f",
    "dlatrz.f",
    "dlatsqr.f",
    "dlauu2.f",
    "dlauum.f",
    "dopgtr.f",
    "dopmtr.f",
    "dorbdb.f",
    "dorbdb1.f",
    "dorbdb2.f",
    "dorbdb3.f",
    "dorbdb4.f",
    "dorbdb5.f",
    "dorbdb6.f",
    "dorcsd.f",
    "dorcsd2by1.f",
    "dorg2l.f",
    "dorg2r.f",
    "dorgbr.f",
    "dorghr.f",
    "dorgl2.f",
    "dorglq.f",
    "dorgql.f",
    "dorgqr.f",
    "dorgr2.f",
    "dorgrq.f",
    "dorgtr.f",
    "dorgtsqr.f",
    "dorgtsqr_row.f",
    "dorhr_col.f",
    "dorm22.f",
    "dorm2l.f",
    "dorm2r.f",
    "dormbr.f",
    "dormhr.f",
    "dorml2.f",
    "dormlq.f",
    "dormql.f",
    "dormqr.f",
    "dormr2.f",
    "dormr3.f",
    "dormrq.f",
    "dormrz.f",
    "dormtr.f",
    "dpbcon.f",
    "dpbequ.f",
    "dpbrfs.f",
    "dpbstf.f",
    "dpbsv.f",
    "dpbsvx.f",
    "dpbtf2.f",
    "dpbtrf.f",
    "dpbtrs.f",
    "dpftrf.f",
    "dpftri.f",
    "dpftrs.f",
    "dpocon.f",
    "dpoequ.f",
    "dpoequb.f",
    "dporfs.f",
    "dposv.f",
    "dposvx.f",
    "dpotf2.f",
    "dpotrf.f",
    "dpotrf2.f",
    "dpotri.f",
    "dpotrs.f",
    "dppcon.f",
    "dppequ.f",
    "dpprfs.f",
    "dppsv.f",
    "dppsvx.f",
    "dpptrf.f",
    "dpptri.f",
    "dpptrs.f",
    "dpstf2.f",
    "dpstrf.f",
    "dptcon.f",
    "dpteqr.f",
    "dptrfs.f",
    "dptsv.f",
    "dptsvx.f",
    "dpttrs.f",
    "dptts2.f",
    "drscl.f",
    "dsb2st_kernels.f",
    "dsbev.f",
    "dsbev_2stage.f",
    "dsbevd.f",
    "dsbevd_2stage.f",
    "dsbevx.f",
    "dsbevx_2stage.f",
    "dsbgst.f",
    "dsbgv.f",
    "dsbgvd.f",
    "dsbgvx.f",
    "dsbtrd.f",
    "dsfrk.f",
    "dsgesv.f",
    "dspcon.f",
    "dspev.f",
    "dspevd.f",
    "dspevx.f",
    "dspgst.f",
    "dspgv.f",
    "dspgvd.f",
    "dspgvx.f",
    "dsposv.f",
    "dsprfs.f",
    "dspsv.f",
    "dspsvx.f",
    "dsptrd.f",
    "dsptrf.f",
    "dsptri.f",
    "dsptrs.f",
    "dstegr.f",
    "dstein.f",
    "dstemr.f",
    "dstev.f",
    "dstevd.f",
    "dstevr.f",
    "dstevx.f",
    "dsycon.f",
    "dsycon_3.f",
    "dsycon_rook.f",
    "dsyconv.f",
    "dsyconvf.f",
    "dsyconvf_rook.f",
    "dsyequb.f",
    "dsyev.f",
    "dsyev_2stage.f",
    "dsyevd.f",
    "dsyevd_2stage.f",
    "dsyevr.f",
    "dsyevr_2stage.f",
    "dsyevx.f",
    "dsyevx_2stage.f",
    "dsygs2.f",
    "dsygst.f",
    "dsygv.f",
    "dsygv_2stage.f",
    "dsygvd.f",
    "dsygvx.f",
    "dsyrfs.f",
    "dsysv.f",
    "dsysv_aa.f",
    "dsysv_aa_2stage.f",
    "dsysv_rk.f",
    "dsysv_rook.f",
    "dsysvx.f",
    "dsyswapr.f",
    "dsytd2.f",
    "dsytf2.f",
    "dsytf2_rk.f",
    "dsytf2_rook.f",
    "dsytrd.f",
    "dsytrd_2stage.f",
    "dsytrd_sb2st.F",
    "dsytrd_sy2sb.f",
    "dsytrf.f",
    "dsytrf_aa.f",
    "dsytrf_aa_2stage.f",
    "dsytrf_rk.f",
    "dsytrf_rook.f",
    "dsytri.f",
    "dsytri2.f",
    "dsytri2x.f",
    "dsytri_3.f",
    "dsytri_3x.f",
    "dsytri_rook.f",
    "dsytrs.f",
    "dsytrs2.f",
    "dsytrs_3.f",
    "dsytrs_aa.f",
    "dsytrs_aa_2stage.f",
    "dsytrs_rook.f",
    "dtbcon.f",
    "dtbrfs.f",
    "dtbtrs.f",
    "dtfsm.f",
    "dtftri.f",
    "dtfttp.f",
    "dtfttr.f",
    "dtgevc.f",
    "dtgex2.f",
    "dtgexc.f",
    "dtgsen.f",
    "dtgsja.f",
    "dtgsna.f",
    "dtgsy2.f",
    "dtgsyl.f",
    "dtpcon.f",
    "dtplqt.f",
    "dtplqt2.f",
    "dtpmlqt.f",
    "dtpmqrt.f",
    "dtpqrt.f",
    "dtpqrt2.f",
    "dtprfb.f",
    "dtprfs.f",
    "dtptri.f",
    "dtptrs.f",
    "dtpttf.f",
    "dtpttr.f",
    "dtrcon.f",
    "dtrevc.f",
    "dtrevc3.f",
    "dtrexc.f",
    "dtrrfs.f",
    "dtrsen.f",
    "dtrsna.f",
    "dtrsyl.f",
    "dtrsyl3.f",
    "dtrti2.f",
    "dtrtri.f",
    "dtrtrs.f",
    "dtrttf.f",
    "dtrttp.f",
    "dtzrzf.f",
    "iladlc.f",
    "iladlr.f",
    "slag2d.f",
]
