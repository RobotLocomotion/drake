
SUBROUTINE fout(unitnum, messag)
  !DEC$ ATTRIBUTES STDCALL :: fout
  !DEC$ ATTRIBUTES REFERENCE :: fout
  INTEGER :: unitnum
  CHARACTER*(*) :: messag

  WRITE (unitnum, '(a)') messag
 10   format (a)
  RETURN
END SUBROUTINE fout
