#!/usr/bin/env python
import os
import rospy
from Tkinter import *
import tkFileDialog
import tkMessageBox

class Application(Frame):
    
    current_directory = ''
    img_last_folder = "/home"
    img_folder_list = []
    img_listbox = 0
    fileMenu = 0
    menubar = 0
    
    def say_hi(self):
        print "hi there, everyone!"

    def createWidgets(self):
                
        #setting up the window
        self.master.title("Annotation")
        self.master.geometry('%dx%d+%d+%d' % (self.getScreenWidth() / 2, self.getScreenHeight() / 2, self.getScreenWidth() / 2 - self.getScreenWidth() / 4, self.getScreenHeight() / 2 - self.getScreenHeight() / 4))
        self.master.resizable(0,0)        
        
        #setting up menu bar
        menubar = Menu(self.master)
        self.master.config(menu=menubar)
        
        fileMenu = Menu(menubar)
        fileMenu.add_command(label="Exit", command=self.onExit)
        fileMenu.add_command(label="Add Image Source", command=self.addMBTNonOpen)
        menubar.add_cascade(label="File", menu=fileMenu)
        
        #setting up listbox
        self.img_listbox = Listbox(self.master)
        self.img_listbox.pack(side=LEFT, fill=Y, expand=0)
        
        #setting up listbox delete button
        
        
        self.QUIT = Button(self)
        self.QUIT["text"] = "QUIT"
        self.QUIT["fg"]   = "red"
        self.QUIT["command"] =  self.quit

        self.QUIT.pack({"side": "left"})

        self.hi_there = Button(self)
        self.hi_there["text"] = "Hello",
        self.hi_there["command"] = self.say_hi

        self.hi_there.pack({"side": "left"})
        
        self.testBTN = Button(self, text="Test", command=self.testBTNCB)
        self.testBTN.pack()
    
    #add image source callback
    def addMBTNonOpen(self):
        self.img_last_folder = tkFileDialog.askdirectory(parent=root,initialdir=self.img_last_folder,title='Please select image source directory')
        if self.img_last_folder not in self.img_folder_list:
            self.img_folder_list.append(self.img_last_folder)
        
        self.updateListBox(self.img_folder_list)
        #self.img_listbox.insert(0, self.img_last_folder)
        
        print self.img_folder_list
 
    def updateListBox(self, list_original):
        self.img_listbox.delete(0, END)
        for item in list_original:
            self.img_listbox.insert(END, item)
            
    def removeFromListBox(self, index):
        self.img_folder_list.remove(self.img_folder_list[index])
        self.updateListBox(self.img_folder_list)
    
    def onExit(self):
        self.quit()
        
    def testBTNCB(self):
        tkMessageBox.showinfo("Test Button", "test button is pressed")
        tkMessageBox.showerror("Error", "Error Message")
    def __init__(self, master=None):
        Frame.__init__(self, master)
        self.pack()
        self.createWidgets()
    
    def getScreenSizeStr(self, debug):
        screen_width = getScreenWidth()
        screen_height = getScreenHeight()
    
        if debug == True:
            print str(screen_width / 2) + "x" + str(screen_height/2)
        
        return str(screen_width / 2) + "x" + str(screen_height/2)

    def getScreenWidth(self):
        return root.winfo_screenwidth()
    
    def getScreenHeight(self):
        return root.winfo_screenheight()

if __name__ == "__main__":
    root = Tk()

    app = Application(master=root)
   
    
    app.mainloop()
    root.destroy()
    