#!/usr/bin/env python
# -*- coding: UTF-8 -*-
#
# generated by wxGlade 0.7.1 on Sun Nov 11 17:57:19 2018
#

import wx

# begin wxGlade: dependencies
import gettext
# end wxGlade

# begin wxGlade: extracode
# end wxGlade


class MyFrame(wx.Frame):
    def __init__(self, *args, **kwds):
        # begin wxGlade: MyFrame.__init__
        wx.Frame.__init__(self, *args, **kwds)
        self.test = wx.Button(self, wx.ID_ANY, _("test"))

        self.__set_properties()
        self.__do_layout()

        self.Bind(wx.EVT_BUTTON, self.testBTNPressed, self.test)
        # end wxGlade

    def __set_properties(self):
        # begin wxGlade: MyFrame.__set_properties
        self.SetTitle(_("frame_1"))
        # end wxGlade

    def __do_layout(self):
        # begin wxGlade: MyFrame.__do_layout
        sizer_1 = wx.BoxSizer(wx.VERTICAL)
        sizer_1.Add(self.test, 0, 0, 0)
        self.SetSizer(sizer_1)
        sizer_1.Fit(self)
        self.Layout()
        # end wxGlade

    def testBTNPressed(self, event):  # wxGlade: MyFrame.<event_handler>
        print "Event handler 'testBTNPressed' not implemented!"
        event.Skip()

# end of class MyFrame
class mainClass(wx.App):
    def OnInit(self):
        frame_1 = MyFrame(None, wx.ID_ANY, "")
        self.SetTopWindow(frame_1)
        frame_1.Show()
        return True

# end of class mainClass

if __name__ == "__main__":
    gettext.install("test") # replace with the appropriate catalog name

    test = mainClass(0)
    test.MainLoop()