del *.bak /s
del *.ddk /s
del *.edk /s
del *.lst /s
del *.lnp /s
del *.mpf /s
del *.mpj /s
del *.obj /s
del *.Output/Release_Lib /s
del *.omf /s
del *.hex /s
::del *.h /s
del *.bin /s
del *.elf /s
::del *.opt /s  ::不允许删除JLINK的设置
del *.plg /s
del *.rpt /s
del *.tmp /s
del *.__i /s
del *.crf /s
del *.o /s
del *.d /s
del *.axf /s
del *.tra /s
del *.dep /s           
del JLinkLog.txt /s
del *._ac /s
del *._ia /s
del *.lst /s
::del *.sct /s

RD /S /Q "Listings"
RD /S /Q "Objects"
RD /S /Q "RTE"
RD /S /Q "OBJ"
RD /S /Q "Output"
del *.iex /s
del *.Administrator /s
del *.htm /s
del *.map /s
del *.dell /s
del *.19513 /s
exit
