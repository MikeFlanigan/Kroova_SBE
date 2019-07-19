try:
    from Tkinter import *
    import Tkinter.font
except ImportError:
    from tkinter import *
    import tkinter.font

win = Tk() # main window
##win.overrideredirect(True)
##win.geometry("{0}x{1}+0+0".format(win.winfo_screenwidth(), win.winfo_screenheight()))
win.focus_set()  # <-- move focus to this widget

# useful HEX color defs
white = '#000000' # actually black :)
black = '#FFFFFF' # actually white
green = '#000000' # Quick color change
##green = '#09FF00'
red = '#FF0000'
yellow = '#FFEB00'

def close():
    win.destroy()   # Close GUI




def clock_tick():
    win.after(100, clock_tick)
##    update_foil_rake_display(msg_queue)
##    show_connection_status()

##############################################################
    # GUI set up
##############################################################

'''This class configures and populates the winlevel window.
win is the winlevel containing window.'''
_bgcolor = '#d9d9d9'  # X11 color: 'gray85'
_fgcolor = '#000000'  # X11 color: 'black'
_compcolor = '#d9d9d9' # X11 color: 'gray85'
_ana1color = '#d9d9d9' # X11 color: 'gray85' 
_ana2color = '#d9d9d9' # X11 color: 'gray85'
font10 = "-family {Segoe UI} -size 30 -weight normal -slant "  \
    "roman -underline 0 -overstrike 0"
font11 = "-family {Segoe UI} -size 20 -weight normal -slant "  \
    "roman -underline 0 -overstrike 0"
font12 = "-family {Segoe UI} -size 30 -weight bold -slant "  \
    "roman -underline 1 -overstrike 0"
font15 = "-family {Segoe UI} -size 24 -weight bold -slant "  \
    "roman -underline 0 -overstrike 0"
font16 = "-family {Segoe UI} -size 48 -weight bold -slant "  \
    "roman -underline 0 -overstrike 0"
font17 = "-family {Segoe UI} -size 23 -weight bold -slant "  \
    "roman -underline 0 -overstrike 0"
font18 = "-family {Segoe UI} -size 21 -weight bold -slant "  \
    "roman -underline 0 -overstrike 0"
font9 = "-family {Segoe UI} -size 20 -weight bold -slant roman"  \
    " -underline 0 -overstrike 0"

win.geometry("2400x1261+-1394+-14")
win.title("New winlevel")
win.configure(background="#d9d9d9")

Button1 = Button(win,command=close)
Button1.place(relx=0.86, rely=0.01, relheight=0.05, relwidth=0.1)
Button1.configure(activebackground=white)
Button1.configure(activeforeground="#000000")
Button1.configure(background=black)
Button1.configure(disabledforeground="#a3a3a3")
Button1.configure(font=font9)
Button1.configure(foreground=white)
Button1.configure(highlightbackground="#d9d9d9")
Button1.configure(borderwidth=6)
Button1.configure(highlightcolor=white)
Button1.configure(pady="0")
Button1.configure(text='''Quit''')
Button1.configure(width=217)

Label1 = Label(win)
Label1.place(relx=0.05, rely=0.44, height=126, width=242)
Label1.configure(background="#d9d9d9")
Label1.configure(disabledforeground="#a3a3a3")
Label1.configure(font=font10)
Label1.configure(foreground="#000000")
Label1.configure(text='''Label''')
Label1.configure(width=242)

Button1_6 = Button(win)
Button1_6.place(relx=0.08, rely=0.05, height=93, width=156)
Button1_6.configure(activebackground="#d9d9d9")
Button1_6.configure(activeforeground="#000000")
Button1_6.configure(background="#d9d9d9")
Button1_6.configure(disabledforeground="#a3a3a3")
Button1_6.configure(font=font11)
Button1_6.configure(foreground="#000000")
Button1_6.configure(highlightbackground="#d9d9d9")
Button1_6.configure(highlightcolor="black")
Button1_6.configure(pady="0")
Button1_6.configure(text='''+50''')
Button1_6.configure(width=156)

Label1_9 = Label(win)
Label1_9.place(relx=0.04, rely=0.36, height=86, width=372)
Label1_9.configure(activebackground="#f9f9f9")
Label1_9.configure(activeforeground="black")
Label1_9.configure(background="#d9d9d9")
Label1_9.configure(disabledforeground="#a3a3a3")
Label1_9.configure(font=font10)
Label1_9.configure(foreground="#000000")
Label1_9.configure(highlightbackground="#d9d9d9")
Label1_9.configure(highlightcolor="black")
Label1_9.configure(text='''Target RH [mm]''')
Label1_9.configure(width=372)

Button1_7 = Button(win)
Button1_7.place(relx=0.08, rely=0.15, height=93, width=156)
Button1_7.configure(activebackground="#d9d9d9")
Button1_7.configure(activeforeground="#000000")
Button1_7.configure(background="#d9d9d9")
Button1_7.configure(disabledforeground="#a3a3a3")
Button1_7.configure(font=font11)
Button1_7.configure(foreground="#000000")
Button1_7.configure(highlightbackground="#d9d9d9")
Button1_7.configure(highlightcolor="black")
Button1_7.configure(pady="0")
Button1_7.configure(text='''+25''')

Button1_8 = Button(win)
Button1_8.place(relx=0.08, rely=0.26, height=93, width=156)
Button1_8.configure(activebackground="#d9d9d9")
Button1_8.configure(activeforeground="#000000")
Button1_8.configure(background="#d9d9d9")
Button1_8.configure(disabledforeground="#a3a3a3")
Button1_8.configure(font=font11)
Button1_8.configure(foreground="#000000")
Button1_8.configure(highlightbackground="#d9d9d9")
Button1_8.configure(highlightcolor="black")
Button1_8.configure(pady="0")
Button1_8.configure(text='''+5''')
Button1_8.configure(width=156)

Button1_9 = Button(win)
Button1_9.place(relx=0.08, rely=0.56, height=93, width=156)
Button1_9.configure(activebackground="#d9d9d9")
Button1_9.configure(activeforeground="#000000")
Button1_9.configure(background="#d9d9d9")
Button1_9.configure(disabledforeground="#a3a3a3")
Button1_9.configure(font=font11)
Button1_9.configure(foreground="#000000")
Button1_9.configure(highlightbackground="#d9d9d9")
Button1_9.configure(highlightcolor="black")
Button1_9.configure(pady="0")
Button1_9.configure(text='''-5''')

Button1_8 = Button(win)
Button1_8.place(relx=0.08, rely=0.67, height=93, width=156)
Button1_8.configure(activebackground="#d9d9d9")
Button1_8.configure(activeforeground="#000000")
Button1_8.configure(background="#d9d9d9")
Button1_8.configure(disabledforeground="#a3a3a3")
Button1_8.configure(font=font11)
Button1_8.configure(foreground="#000000")
Button1_8.configure(highlightbackground="#d9d9d9")
Button1_8.configure(highlightcolor="black")
Button1_8.configure(pady="0")
Button1_8.configure(text='''-25''')

Button1_7 = Button(win)
Button1_7.place(relx=0.08, rely=0.8, height=93, width=156)
Button1_7.configure(activebackground="#d9d9d9")
Button1_7.configure(activeforeground="#000000")
Button1_7.configure(background="#d9d9d9")
Button1_7.configure(disabledforeground="#a3a3a3")
Button1_7.configure(font=font11)
Button1_7.configure(foreground="#000000")
Button1_7.configure(highlightbackground="#d9d9d9")
Button1_7.configure(highlightcolor="black")
Button1_7.configure(pady="0")
Button1_7.configure(text='''-50''')

Label1_8 = Label(win)
Label1_8.place(relx=0.22, rely=0.36, height=86, width=392)
Label1_8.configure(activebackground="#f9f9f9")
Label1_8.configure(activeforeground="black")
Label1_8.configure(background="#d9d9d9")
Label1_8.configure(disabledforeground="#a3a3a3")
Label1_8.configure(font=font10)
Label1_8.configure(foreground="#000000")
Label1_8.configure(highlightbackground="#d9d9d9")
Label1_8.configure(highlightcolor="black")
Label1_8.configure(text='''Roll Avg Window''')
Label1_8.configure(width=392)

Label1_9 = Label(win)
Label1_9.place(relx=0.24, rely=0.44, height=126, width=242)
Label1_9.configure(activebackground="#f9f9f9")
Label1_9.configure(activeforeground="black")
Label1_9.configure(background="#d9d9d9")
Label1_9.configure(disabledforeground="#a3a3a3")
Label1_9.configure(font=font10)
Label1_9.configure(foreground="#000000")
Label1_9.configure(highlightbackground="#d9d9d9")
Label1_9.configure(highlightcolor="black")
Label1_9.configure(text='''Label''')

Button1_7 = Button(win)
Button1_7.place(relx=0.27, rely=0.05, height=93, width=156)
Button1_7.configure(activebackground="#d9d9d9")
Button1_7.configure(activeforeground="#000000")
Button1_7.configure(background="#d9d9d9")
Button1_7.configure(disabledforeground="#a3a3a3")
Button1_7.configure(font=font11)
Button1_7.configure(foreground="#000000")
Button1_7.configure(highlightbackground="#d9d9d9")
Button1_7.configure(highlightcolor="black")
Button1_7.configure(pady="0")
Button1_7.configure(text='''+15''')

Button1_8 = Button(win)
Button1_8.place(relx=0.27, rely=0.16, height=93, width=156)
Button1_8.configure(activebackground="#d9d9d9")
Button1_8.configure(activeforeground="#000000")
Button1_8.configure(background="#d9d9d9")
Button1_8.configure(disabledforeground="#a3a3a3")
Button1_8.configure(font=font11)
Button1_8.configure(foreground="#000000")
Button1_8.configure(highlightbackground="#d9d9d9")
Button1_8.configure(highlightcolor="black")
Button1_8.configure(pady="0")
Button1_8.configure(text='''+5''')

Button1_8 = Button(win)
Button1_8.place(relx=0.27, rely=0.26, height=93, width=156)
Button1_8.configure(activebackground="#d9d9d9")
Button1_8.configure(activeforeground="#000000")
Button1_8.configure(background="#d9d9d9")
Button1_8.configure(disabledforeground="#a3a3a3")
Button1_8.configure(font=font11)
Button1_8.configure(foreground="#000000")
Button1_8.configure(highlightbackground="#d9d9d9")
Button1_8.configure(highlightcolor="black")
Button1_8.configure(pady="0")
Button1_8.configure(text='''+1''')

Button1_8 = Button(win)
Button1_8.place(relx=0.27, rely=0.56, height=93, width=156)
Button1_8.configure(activebackground="#d9d9d9")
Button1_8.configure(activeforeground="#000000")
Button1_8.configure(background="#d9d9d9")
Button1_8.configure(disabledforeground="#a3a3a3")
Button1_8.configure(font=font11)
Button1_8.configure(foreground="#000000")
Button1_8.configure(highlightbackground="#d9d9d9")
Button1_8.configure(highlightcolor="black")
Button1_8.configure(pady="0")
Button1_8.configure(text='''+15''')

Button1_8 = Button(win)
Button1_8.place(relx=0.27, rely=0.67, height=93, width=156)
Button1_8.configure(activebackground="#d9d9d9")
Button1_8.configure(activeforeground="#000000")
Button1_8.configure(background="#d9d9d9")
Button1_8.configure(disabledforeground="#a3a3a3")
Button1_8.configure(font=font11)
Button1_8.configure(foreground="#000000")
Button1_8.configure(highlightbackground="#d9d9d9")
Button1_8.configure(highlightcolor="black")
Button1_8.configure(pady="0")
Button1_8.configure(text='''+15''')
Button1_8.configure(width=156)

Button1_8 = Button(win)
Button1_8.place(relx=0.27, rely=0.8, height=93, width=156)
Button1_8.configure(activebackground="#d9d9d9")
Button1_8.configure(activeforeground="#000000")
Button1_8.configure(background="#d9d9d9")
Button1_8.configure(disabledforeground="#a3a3a3")
Button1_8.configure(font=font11)
Button1_8.configure(foreground="#000000")
Button1_8.configure(highlightbackground="#d9d9d9")
Button1_8.configure(highlightcolor="black")
Button1_8.configure(pady="0")
Button1_8.configure(text='''+15''')

Label1_9 = Label(win)
Label1_9.place(relx=0.4, rely=0.36, height=86, width=392)
Label1_9.configure(activebackground="#f9f9f9")
Label1_9.configure(activeforeground="black")
Label1_9.configure(background="#d9d9d9")
Label1_9.configure(disabledforeground="#a3a3a3")
Label1_9.configure(font=font10)
Label1_9.configure(foreground="#000000")
Label1_9.configure(highlightbackground="#d9d9d9")
Label1_9.configure(highlightcolor="black")
Label1_9.configure(text='''P gain''')

Label1_10 = Label(win)
Label1_10.place(relx=0.43, rely=0.43, height=126, width=242)
Label1_10.configure(activebackground="#f9f9f9")
Label1_10.configure(activeforeground="black")
Label1_10.configure(background="#d9d9d9")
Label1_10.configure(disabledforeground="#a3a3a3")
Label1_10.configure(font=font10)
Label1_10.configure(foreground="#000000")
Label1_10.configure(highlightbackground="#d9d9d9")
Label1_10.configure(highlightcolor="black")
Label1_10.configure(text='''Label''')
Label1_10.configure(width=242)

##############################################################
    # End of GUI set up
##############################################################


clock_tick() # run clock tick function once to initialize


win.mainloop() # Loops forever
